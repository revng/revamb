#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

import json
import re
import yaml

class SafeLoaderIgnoreUnknown(yaml.SafeLoader):
    def ignore_unknown(self, node):
        return None 

SafeLoaderIgnoreUnknown.add_constructor(None, SafeLoaderIgnoreUnknown.ignore_unknown)

class MetaAddressRemapper:
    def __init__(self):
        self.addresses = set()

    def handle(self, value):
        if (isinstance(value, str)
            and value.startswith("0x")
            and ":Code_" in value):

            self.addresses.add(value)

    def collect(self, value):
        if isinstance(value, dict):
            for k, v in value.items():
                self.handle(k)
                self.collect(v)

        elif isinstance(value, list):
            for v in value:
                self.collect(v)

        else:
            self.handle(value)

    def apply_replacement(self, value):
        return (self.replacements[value]
                if (isinstance(value, str)
                    and value in self.replacements)
                else value)

    def replace(self, value):
        if isinstance(value, dict):
            new_values = {self.apply_replacement(k) :
                          self.apply_replacement(v)
                          for k, v
                          in value.items()}
            value.clear()
            value.update(new_values)

            for k, v in value.items():
                self.replace(v)

        elif isinstance(value, list):
            new_values = [self.apply_replacement(v) for v in value]
            value.clear()
            for new_value in new_values:
                value.append(new_value)

            for v in value:
                self.replace(v)

    def rewrite(self, value):
        self.replacements = {v: str(i)
                             for i, v
                             in enumerate(sorted(self.addresses))}
        self.replace(value)

        return value

def remap_metaaddress(model):
    mar = MetaAddressRemapper()
    mar.collect(model)
    return mar.rewrite(model)

def fetch_text_model(stream):
    prefix = None

    # Find revng.model named metadata
    for line in stream:
        match = re.match("!revng.model = !{!([0-9]*)}", line)
        if match:
            prefix = "!" + match.groups(1)[0] + " = !{!\""
            break

    # Early exit if not found
    if not prefix:
        return None

    # Look for associated named metadata
    for line in stream:
        if prefix and line.startswith(prefix):
            text_model = line[len(prefix):-3]

            # Unescape the string
            for escaped in set(re.findall(r"\\[0-9a-fA-F]{2}", text_model)):
                replacement = bytearray.fromhex(escaped[1:]).decode()
                text_model = text_model.replace(escaped, replacement)

            return text_model

    return None

def parse_model(text_model):
    return yaml.load(text_model, Loader=SafeLoaderIgnoreUnknown)

def load_model(stream):
    text_model = fetch_text_model(stream)
    return parse_model(text_model) if text_model else None
