#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

import argparse
import yaml

from dump_model import fetch_text_model, parse_model, remap_metaaddress

def log(message):
    sys.stderr.write(message + "\n")

def main():
    parser = argparse.ArgumentParser(description="Extract and process rev.ng model.")
    parser.add_argument("--json",
                        action="store_true",
                        help="Dump as JSON.")
    parser.add_argument("--remap",
                        action="store_true",
                        help="Remap MetaAddresses. Implies --json.")
    args = parser.parse_args()

    if args.remap:
        args.json = True

    text_model = fetch_text_model(sys.stdin)
    if text_model:
        log("Couldn't load model")
        return 1

    if not args.json:
        print(text_model)
        return 0

    # Decode YAML
    model = parse_model(text_model)

    # Remap MetaAddress
    if args.remap:
        model = remap_metaaddress(model)

    # Dump as JSON
    print(json.dumps(model,
                     indent=2,
                     sort_keys=True,
                     check_circular=False))
    return 0
