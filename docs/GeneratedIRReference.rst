***********************
rev.ng Output Reference
***********************

The main goal of rev.ng is to produce an LLVM module reproducing the behavior of
the input program. The module should be compilable and work out of the
box. However, such module contains also a rich set of additional information
recovered during the analysis process that the user can exploit to develop any
kind of analysis he wants.

This document details how to interpret the information present in the generated
module. Please refer to the `LLVM Language Reference Manual`_ for details on the
LLVM language itself.

The various sections of this document will present example to clarify the
presented concepts. All the examples originate from the translation of a simple
program compiled for x86-64:

.. code-block:: c

    int myfunction(void) {
      return 42;
    }

    int _start(void) {
      int a = 42;
      return a + myfunction();
    }

The program has been compiled as follows (note that for this step we are using the x86_64 compiler toolchain provided with orchestra, you can type ``orc install toolchain/x86-64/gcc`` to install it):

.. code-block:: sh

   x86_64-gentoo-linux-musl-gcc -static -nostdlib -O0 -fomit-frame-pointer example.c -o example

Producing the following assembly:

.. code-block:: objdump

    00000000004000e8 <myfunction>:
      4000e8:     mov    eax,0x2a
      4000ed:     ret

    00000000004000ee <_start>:
      4000ee:     sub    rsp,0x10
      4000f2:     mov    DWORD PTR [rsp+0xc],0x2a
      4000fa:     call   4000e8 <myfunction>
      4000ff:     mov    edx,eax
      400101:     mov    eax,DWORD PTR [rsp+0xc]
      400105:     add    eax,edx
      400107:     add    rsp,0x10
      40010b:     ret

And it has been translated as follows:

.. code-block:: sh

   revng lift --debug-info ll example example.ll

Global variables
================

The CPU State Variables
-----------------------

The CPU State Variables (or CSV) are global variables that represent a part of
the CPU. They vary from architecture to architecture and they are created
on-demand, which means that not all modules will have all of them.

Some CSV variables have a name, in particular registers (e.g., ``rsp``), some
others are instead identified by their position within the QEMU data structure
that contains them (e.g., ``state_0x123``). For example:

.. code-block:: llvm

    @pc = global i64 0
    @rsp = global i64 0
    @rax = global i64 0
    @rdx = global i64 0
    @cc_src = global i64 0
    @cc_dst = global i64 0
    @cc_op = global i32 0

``@pc`` represents the program counter (aka ``rip``), ``@rsp`` the stack pointer
register, while ``@rax`` and ``@rdx`` are general purpose registers. The
``@cc_*`` are helper variables used to compute the CPU flags.

CSVs are used by the generated code and by the helper functions. This is also
the reason why they cannot be promoted to local variables in the ``root``
function

Note that since they are global variables, the generated code interacts with
them using load and store operations, which might sound unusual for registers.

Segment variables
-----------------

The translated program expects the memory layout to be exactly as the one in the
original binary. This means that all the segments have to be loaded at the
original addresses. In the generated module, they are encoded as global
variables containing all the data of the segments. These variables have a name
similar to ``.o_permissions_address`` (e.g., ``.o_rx_0x10000``), where
*permissions* it's a string representing what type of accesses are allowed to
that segment (read, execute, write), and *address* is the starting address.

These variables are associated to special sections which will be assigned to the
appropriate virtual address at link-time.

In our example we have single segment, readable and executable:

.. code-block:: llvm

   @.o_rx_0x400000 = constant [344 x i8] c"\7FELF\02\01\01\0...", section ".o_rx_0x400000", align 1

As you can see it is initialized with a copy of the original segment and it's
assigned to the ``.o_rx_0x400000`` section.

Other global variables
----------------------

Apart from CSVs and segment variables, the output module will contain a number
of other global variables, mainly for loading purposes (see ``support.c``). In
the following we report the most relevant ones.

:``.elfheaderhelper``: a variable whose only purpose is to create the
                       ``.elfheaderhelper`` section, which is employed to force
                       an appropriate layout at link-time. It isn't of general
                       interest.
:``e_phentsize``: size of the ELF program header structure of the input binary.
:``e_phnum``: number of ELF program headers in the input binary.
:``phdr_address``: virtual address where the ELF program headers are loaded.

For more information on the ELF program headers, see ``man elf``.  In the
example program we have three program headers of 56 bytes, loaded at
``0x400040``:

.. code-block:: llvm

    @.elfheaderhelper = constant i8 0, section ".elfheaderhelper", align 1
    @e_phentsize = constant i64 56
    @e_phnum = constant i64 7
    @phdr_address = constant i64 4194368


Input architecture description
==============================

The generated module also contains a *named metadata node*:
``revng.input.architecture``. Currently, it's composed of a metadata tuple with
two values:

:``string ArchitectureName``: the name of the input architecture.
:``u32 InstructionAlignment``: alignment of instructions, for example in ARM
                               instructions have an alignment of 4 bytes, while
                               the alignment for x86 architectures is 1 byte.
:``u32 DelaySlotSize``: the size, in number of instructions of the delay slot of
                        the input architecture.
:``string PCRegisterName``: the name of the CSV representing the program counter.
:``string SPRegisterName``: the name of the CSV representing the stack pointer.
:``string[] ABIRegisters``: list of name of the CSV involved in the ABI, and
                            that, therefore need to be serialized before passing
                            from the translated realm to the native realm and
                            viceversa.

Here's how this information appears in our example:

.. code-block:: llvm

    !revng.input.architecture = !{!1}
    !1 = !{!"x86_64", i32 1, i32 0, !"pc", !"rsp", !2}
    !2 = !{!"rax", !"rbx", !"rcx", !"rdx", !"rbp", ... }

x86-64 has no instruction alignment requirements, no delay slot and the CSV
representing the program counter and the stack pointer are ``@pc`` and ``@rsp``,
respectively.

The ``root`` function
=====================

This section describes how the function collecting all the translated code is
organized. This function is known as the ``root`` function:

.. code-block:: llvm

    define void @root(i64) {
      ; ...
    }

The ``root`` function takes a single argument, which is a pointer to the stack
that the translated program has to use. This stack must have been properly set
up by the caller, for more information see `FromIRToExecutable.rst`_.

First of all, the ``root`` function must set up two key CSVs: the stack pointer
and the program counter:

.. code-block:: llvm

    define void @root(i64) {
    entrypoint:
      ; ...
      store i64 4194542, i64* @pc
      store i64 %0, i64* @rsp
      ; ...
    }

The program counter is obtained from the entry point of the input program and
it's therefore statically available, while the stack pointer (the ``rsp``
register in x86-64), is taken from the first argument of the ``root`` function.

The dispatcher
--------------

The first set of basic blocks are related to the dispatcher. Every time we have
an indirect branch for which we were not able to exhaustively enumerate all the
possible targets, we jump to the *dispatcher*. The dispatcher, maps (with a huge
``switch`` statement) the starting address of each basic block A in the input
program to the first basic block containing the code generated due to A.

:``dispatcher.entry``: the body of the dispatcher. Contains the ``switch``
                       statement. If the requested address has not been
                       translated, execution is diverted to
                       ``dispatcher.external``.
:``dispatcher.external``: the value of the program counter doesn't match any of
                          the translated ones. This basic block checks whether
                          the value falls within an executable segment of the
                          input program (using the ``is_executable`` function
                          from ``support.c``. If it is, then rev.ng was not able
                          to properly identify this basic block and we jump to
                          ``dispatcher.default``. Otherwise, the program counter
                          might be actually invalid or it could belong to a
                          function in a dynamic library. In this case, we simply
                          leave the translated realm and jump there.
:``dispatcher.default``: calls the ``unknownPC`` function, whose definition is
                         left to the user. The default implementation in
                         ``support.c`` aborts the program execution.
:``anypc``: handles the situation in which we were not able to fully enumerate
            all the possible jump targets of an indirect jump. Typically will
            just jump to ``dispatcher.entry``.
:``unexpectedpc``: handles the situation in which we thought we were able to
                   enumerate all the possible jump targets, but an unexpected
                   program counter was requested. This indicates the presence of
                   a bug. It can either try to proceed with execution going to
                   ``dispatcher.entry`` or simply abort.

The very first basic block is ``entrypoint``. Its main purpose is to create all
the required local variables (``alloca`` instructions) and ensure that all the
basic blocks are reachable. In fact, it is terminated by a ``switch``
instruction which makes all the previously mentioned basic blocks reachable. This
ensures that we can compute a proper dominator tree and no basic blocks are
collected as dead code.

Here's how it looks like in our example:

.. code-block:: llvm

    define void @root(i64) !dbg !4 {
    entrypoint:
      %1 = alloca i64
      %2 = bitcast i64* %1 to i8*
      store i64 4194542, i64* @pc
      store i64 %0, i64* @rsp
      switch i8 0, label %dispatcher.entry [
        i8 1, label %anypc
        i8 2, label %unexpectedpc
      ]

    dispatcher.entry:                                 ; preds = %unexpectedpc, %anypc, %bb.myfunction, %bb._start.0x11, %entrypoint
      %3 = load i64, i64* @pc
      switch i64 %3, label %dispatcher.external [
        i64 4194536, label %bb.myfunction
        i64 4194542, label %bb._start
        i64 4194559, label %bb._start.0x11
      ], !revng.block.type !1

    dispatcher.external:                              ; preds = %dispatcher.entry
      %45 = load i64, i64* @pc
      %46 = call i1 @is_executable(i64 %45), !dbg !211
      br i1 %46, label %dispatcher.default, label %setjmp

    dispatcher.default:                               ; preds = %dispatcher.entry
      call void @unknownPC()
      unreachable

    anypc:                                            ; preds = %entrypoint
      br label %dispatcher.entry, !revng.block.type !2

    unexpectedpc:                                     ; preds = %entrypoint
      br label %dispatcher.entry, !revng.block.type !3

    ; ...

    }

As you can see, we have three jump targets: ``myfunction``, ``_start`` and
``_start+0x11`` (the return address after the function call). In this specific
example we decide to divert execution to the dispatcher both in ``anypc`` and
``unexpectedpc``.

The translated basic blocks
---------------------------

The rest of the function is composed by basic blocks containing the translated
code. If symbols are available in the input binary, each basic block has name in
the form ``bb.closest_symbol.distance`` (e.g., ``bb.main.0x4`` means 4 bytes
after the symbol ``main``). Otherwise the name is simply in the form
``bb.absolute_address`` (e.g., ``bb.0x400000``).

In our example we have three basic blocks:

.. code-block:: llvm

    define void @root(i64) {
    ; ...

    bb._start:            ; preds = %dispatcher.entry, %entrypoint
      ; ...

    bb._start.0x11:       ; preds = %dispatcher.entry
      ; ...

    bb.myfunction:        ; preds = %dispatcher.entry, %bb._start
      ; ...

    }

Debug metadata
--------------

Each instruction we generate is associated with three types of metadata:

:dbg: LLVM debug metadata, used to be able to step through the generated LLVM IR
      (or input assembly or tiny code).
:oi: *original instruction* metadata, contains a pair of elements. The former
     element is a reference to a string global variable containing the
     disassembled input instruction that generated the current instruction. The
     latter element is an integer representing the program counter associated
     with that instruction.
:pi: *portable tiny code instruction* metadata, contains a string representing
     the textual representation of the TCG instruction that generated the
     current instruction.

Note: some optimizations passes might remove the metadata.

For debugging purposes, the generated LLVM IR contains comments with information
derived from these metadata.

As an example, let's see the first instruction of ``myfunction``, ``mov
eax,0x2a``:

.. code-block:: llvm

    @disam_myfunction = internal constant [38 x i8] c"0x00000000004000e8:  mov    eax,0x2a\0A\00"

    define void @root(i64) {

    ; ...

    bb.myfunction:                                    ; preds = %dispatcher.entry, %bb._start
      ; 0x00000000004000e8:  mov    eax,0x2a

      ; movi_i64 tmp0,$0x2a
      ; ext32u_i64 rax,tmp0
      store i64 42, i64* @rax, !dbg !135, !oi !133, !pi !136

      ; ...

    }

    ; ...

    !4 = distinct !DISubprogram(name: "root", ...)
    !133 = !{i8* getelementptr inbounds ([38 x i8], [38 x i8]* @disam_myfunction, i32 0, i32 0), i64 4194480}
    !134 = distinct !{!"movi_i64 tmp0,$0x2a\0A"}
    !135 = !DILocation(line: 244, scope: !4)
    !136 = distinct !{!"ext32u_i64 rax,tmp0,\0A"}

The ``!dbg`` metadata points to a ``DILocation`` object, which tells us that
we're at line 244 within the ``root`` function. This information will allow the
debugger (e.g., ``gdb``) to perform step-by-step debugging. ``!oi`` points to a
metadata node containing a reference to ``@disasm_myfcuntion``, a global
variable containing the disassembled instruction that lead to generate this
instruction and its address (``4194536``). Finally, ``!pi`` points to the TCG
instruction leading to the creation of this instruction.

Above the instruction, we also have comments reporting the corresponding
original and TCG instructions.

Delimiting generated code
-------------------------

The code generated due to a certain input instruction is delimited by calls to a
marker function ``newpc``. This function takes the following arguments plus a set
of variadic arguments:

:u64 Address: the address of the instruction leading to the generation of the
              code coming after the call of ``newpc``.
:u64 InstructionSize: the size of the instruction at ``Address``.
:u1 isJT: a boolean flag indicating whether the instruction at ``Address`` is a
          jump target or not.
:GlobalVariable Disassembled: a reference to the global variable containing the
                              string representing the disassembled instruction
                              (the same as the ``!oi`` metadata).
:u8 \*LocalVariables: a series of pointer to all the local variables used by
                      this instruction.

The call to ``newpc`` prevents the optimizer to reorder instructions across its
boundaries and perform other optimizations. This is useful during analysis and
for debugging purposes, but to achieve optimal performances all these function
calls should be removed.

Let's see how this works for the ``bb.myfunction`` basic block:

.. code-block:: llvm

    bb.myfunction:                                    ; preds = %dispatcher.entry, %bb._start

      ; 0x00000000004000e8:  mov    eax,0x2a
      call void (i64, i64, i32, i8*, ...) @newpc(i64 4194536, i64 5, i32 1, i8* getelementptr inbounds ([38 x i8], [38 x i8]* @disam_myfunction, i32 0, i32 0)), !oi !55, !pi !56

      ; ...

      ; 0x00000000004000ed:  ret
      call void (i64, i64, i32, i8*, ...) @newpc(i64 4194541, i64 1, i32 0, i8* getelementptr inbounds ([38 x i8], [38 x i8]* @disam_myfunction.0x5, i32 0, i32 0)), !oi !58, !pi !59

      ; ...

As you can see there are two calls to ``newpc``, the first for the ``mov``
instruction at ``0x4000e8`` (5 bytes long) and the second one for the ``ret``
instruction at ``0x4000ed`` (1 byte long). Note that the first instruction is a
jump target, in fact ``newpc``'s third parameter is set to ``1``, unlike the
second call.

The default implementation of this function in ``support.c`` does nothing, but
it can be easily customized for tracing purposes. For instance, it could print
the disassembled instruction before the corresponding translated code is
executed.

Function calls
--------------

rev.ng can detect function calls. The terminator of a basic block can be
considered a function call if it's preceded by a call to a function called
``function_call``. This function takes three parameters:

:BlockAddress Callee: reference to the callee basic block. The target of the
   function call, most likely a function.
:BlockAddress Return: reference to the return basic block. It's the basic block
                      associated with the return address.
:u64 ReturnPC: the return address.
:GlobalVariable LinkRegister: reference to the CSV representing the link
                              register for this specific function call. If null,
                              the return address is stored on the stack.
:GlobalVariable ExternalFunction: reference to the global variable containing a
                                  string of the name of the external (i.e.,
                                  library) function that this function actually
                                  calls.

In our example we had a function call in the ``_start`` basic block:

.. code-block:: llvm

    bb._start:                                        ; preds = %dispatcher.entry, %entrypoint

      ; ...

      ; 0x00000000004000fa:  call   0x4000e8

      ; ...

      store i64 4194536, i64* @pc, !dbg !58, !oi !46, !pi !59
      call void @function_call(i8* blockaddress(@root, %bb.myfunction), i8* blockaddress(@root, %bb._start.0x11), i32 4194559, i64* null, i8* null), !dbg !60
      br label %bb.myfunction, !dbg !61, !revng.func.entry !62, !revng.func.member.of !63

As expected, before the branch instruction representing the function call, we
have a call to ``@function_call``. The first argument is the callee basic block
(``bb.myfunction``), the second argument is the return basic block (``_start+0x11``)
and the third one is the return address (``0x4000ff``). The third argument is
null since in x86-64 the return address is stored on the top of the
stack. Finally, the fourth argument is null since this is not a call to an
external function.

Function boundaries and ABI
---------------------------

rev.ng can identify function boundaries and function arguments:

.. code-block:: sh

   revng opt -S example.ll -detect-abi -o example.functions.ll

This information is encoded in the generated module by associating two types of
metadata (``revng.func.entry`` and ``revng.func.member.of``) to the terminator
instruction of each basic block.

:``revng.func.entry``: denotes that the current basic block is the entry block
   of a certain function. The associated metadata tuple
   contains information describing the function.

   :``string Name``: name of the function.
   :``u64 Address``: address of the function.
   :``string Type``: type of function. See the ``FunctionType`` enumeration.
   :``GlobalVariable[] ClobberedCSV``: list of CSVs that are clobbered by this
      function.
   :``{ GlobalVariable, string, string }[] Arguments``: a list of triples where
      the first entry is a CSV, the second one states its status in terms of
      being an argument (see the ``RegisterArgument`` class) and the third one
      its status in terms of being a return value (see the
      ``FunctionReturnValue`` class).

:``revng.func.member.of``: denotes that the current basic block is part of one
   or more functions. For each function we have a pair containing a reference to
   the ``revng.func.entry`` metadata of the corresponding function along with
   the role of this basic block within that function (see the ``BranchType``
   enumeration).

In our example we had three basic blocks: ``_start``, ``_start+0x11`` and
``myfunction``. Let's consider the first two and see what function they belong
to:

.. code-block:: llvm

    define void @root(i64) !dbg !4 {

    ; ...

    bb._start:                                        ; preds = %dispatcher.entry, %entrypoint
      ; ...
      br label %bb.myfunction, !revng.func.entry !62, !revng.func.member.of !67

    bb._start.0x11:                                   ; preds = %dispatcher.entry
      ; ...
      br label %dispatcher.entry, !revng.func.member.of !69

    ; ...

    }

    ; ...

    !62 = !{!"bb._start", i64 4194486, !"Regular", !63, !64}
    !63 = !{i64* @rax, i64* @rdx}
    !64 = !{!65, !66}
    !65 = !{i64* @rax, !"NoOrDead", !"YesOrDead"}
    !66 = !{i64* @rdx, !"NoOrDead", !"Maybe"}
    !67 = !{!68}
    !68 = !{!62, !"HandledCall"}
    !69 = !{!70}
    !70 = !{!62, !"Return"}

    ; ...

``bb._start`` is marked as the entry point of a ``Regular`` function with the
same name, starting at address ``4194486``, clobbering ``rax`` and ``rdx``. The
function ABI involves two registers (``rax`` and ``rdx``), none of them is an
argument but ``rax`` is definitely a return register.

``bb._start`` and ``bb._start.0x11`` belong to a single function,
``bb._start``. The first basic block ends with a function call, while the second
one ends with a return instruction.

Helper functions
================

Certain features of the input CPU would be too big to be expanded in TCG
instructions by QEMU (and therefore translate them in LLVM IR). For this reason,
calls to *helper functions* are emitted. An example of a helper function is the
function handling a syscall or a floating point division. These functions can
take arguments and can read and modify freely all the CSV.

Helper functions are obtained from QEMU in the form of LLVM IR (e.g.,
``libtinycode-helpers-mips.ll``) and are statically linked by rev.ng before
emitting the module.

The presence of helper functions also import a quite large number of data
structures, which are not directly related to rev.ng's output.

Note that an helper function might be present multiple times with different
suffixes. This happens every time an helper function takes as an argument a
pointer to a CSV: for each different invocation we specialize that callee
function by fixing that argument. In this way, we can deterministically know
which parts of the CPU state is touched by an helper.

Currently, there is no complete documentation of all the helper functions. The
best way to understand which helper function does what, is to create a simple
assembly snippet using a specific feature (e.g., a performing a syscall) and
translate it using rev.ng.

Function isolation pass output reference
========================================

This section of the document aims to describe how to apply the function
isolation pass to a simple example, to describe what to expect as output of this
pass and the assumptions made in the isolation pass.

All the following examples originate from the translation of the simple program
already shown in the beginning of this document.

Once we have applied the translation to the original binary we can apply the
function isolation pass using the appropriate pass:

.. code-block:: sh

    revng opt -S example.ll -detect-abi -isolate -invoke-isolated-functions -o example.isolated.ll

As you can see by comparing the original IR and the one to which the function
isolation pass has been applied the main difference is that, on the basis of the
information recovered by the function boundaries analysis applied by revng, now
the code is organized in different LLVM functions.

As a reference, we can see that the basic block ``bb.myfunction`` that belonged
to the ``root`` function after the isolation is in the LLVM function
``bb.myfunction``.

.. code-block:: llvm

    define void @bb.myfunction() {
    bb.myfunction:
      call void (i64, i64, i32, i8*, ...) @newpc(i64 4194536, i64 5, i32 1, i8* null), !dbg !96, !oi !97, !pi !98
      ; ...
      ret void
    }

Moreover, with this structure, instead of tagging the actual function calls with
a call to ``function_call`` we can place a real LLVM function call to the target
function.
Just after the function call we also add a branch to the identified return
address.

As a reference, take the call to ``my_function``. In the original IR it appeared in
this form:

.. code-block:: llvm

    call void @function_call(i8* blockaddress(@root, %bb.myfunction), i8* blockaddress(@root, %bb._start.0x11), i32 4194559), !dbg !60
    br label %bb.myfunction, !dbg !61, !revng.func.entry !62, !revng.func.member.of !63

Now with the actual call appears like this:

.. code-block:: llvm

    call void @bb.myfunction()
    br label %bb._start.0x11

Always on the basis of the information recovered by the analysis performed by
rev.ng we are able to emit ``ret`` instructions where needed.

As a reference, at the end of the basic block ``bb.myfunction`` the branch to the
dispatcher:

.. code-block:: llvm

    br label %dispatcher.entry, !revng.func.entry !151, !revng.func.member.of !152, !func.return !151

has been substituted by the ``ret`` instruction:

.. code-block:: llvm

    ret void

The fact that we are now not always operating inside the ``root`` function
means that we can't simply branch to the dispatcher when we need it.
For this purpose, we have introduced a custom exception handling mechanism to be
able to restore the execution from the dispatcher when things do not go as
expected.

The main idea is to have a sort of separation between the world of the isolated
functions and the ``root`` function. In this way, as soon as possible after the
start of the execution of the program, we try to jump into the *isolated* world
and continue the execution from there. When we are not anymore able to continue
the execution in the *isolated* world we generate an exception that restores the
execution in the other world.

To do this we need to use the exception handling mechanism provided by the LLVM
framework, modifying it a little bit to suit our needs.

The first thing that we do is substitute the code of each ``revng.func.entry``
block in the ``root`` function with an ``invoke`` instruction that calls the
isolated function.  In our example, examining the ``bb._start`` function, we
substitute the code of the entry block with this:

.. code-block:: llvm

    bb._start:                                        ; preds = %dispatcher.entry
      invoke void @bb._start()
              to label %invoke_return unwind label %catchblock

In this way when we reach a point, inside the body of a function, where we need
the dispatcher we can use the ``_Unwind_RaiseException`` function provided by
``libunwind`` to restore the execution in the ``root`` function, where we take
care of doing the right action to correctly continue the execution (i.e. invoke)
the dispatcher.

Due to implementation details, we do not rely on the standard mechanism used by
the C++ exception handling mechanism. For this reason, the ``catchblock`` is not
used, but we always transfer the execution to the ``invoke_return`` block, and
we then check for the value of ``ExceptionFlag`` for deciding where to transfer
the execution.
After this, we transfer the control flow to the ``dispatcher.entry`` block for
resuming the execution in the correct manner.

We then need a ``function_dispatcher`` that acts as a normal dispatcher but is
used in presence of an indirect function call and assumes the form of an LLVM
function. Obviously, the possible targets are only the function entry blocks,
since it is not possible that a function call requires to jump in the middle of
the code of a function.

We also add an extra check after each call to the ``function_dispatcher`` to
ensure that the program counter value is the one that we expect to have after
the call. This mechanism is useful to avoid errors due to a bad identification
of ``ret`` instructions by the function boundaries analysis.

During the execution of the translated program, when an exception is raised, the
``exception_warning`` helper function is called, and it will print on ``stdout``
useful information about the conditions that caused the exception (e.g. the
current program counter at the moment of the exception, the next program
counter, etc.).

.. _LLVM Language Reference Manual: http://llvm.org/docs/LangRef.html
.. _`FromIRToExecutable.rst`: FromIRToExecutable.rst
