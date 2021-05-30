//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "revng/AutoEnforcer/AutoEnforcerTarget.h"
#include "revng/Enforcers/RevngEnforcers.h"

using namespace Model;
using namespace std;

Granularity Model::RootGranularity("Root Granularity");
Granularity
  Model::FunctionsGranularity("Function Granularity", RootGranularity);

Kind Model::Binary("Binary Kind", &RootGranularity);
Kind Model::Root("Root kind", &RootGranularity);
Kind Model::RootIsolated("Root Isolated Kind", Root, &RootGranularity);
Kind Model::Support("Support Kind", &RootGranularity);
Kind Model::Object("Object File Kind", &RootGranularity);
Kind Model::Translated("Translated Kind", &RootGranularity);
Kind Model::Dead("Dead Kind", &RootGranularity);

Kind Model::CFepper("CFepper Kind", &FunctionsGranularity);
Kind Model::Isolated("Isolated Kind", &FunctionsGranularity);
Kind Model::ABIEnforced("ABIEnforced Kind", Isolated, &FunctionsGranularity);
