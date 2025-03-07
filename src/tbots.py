#!/opt/tbotspython/bin/python3

import os
import sys
import iterfzf
import itertools
from subprocess import PIPE, run
import argparse
from thefuzz import process

# thefuzz is a fuzzy string matcher in python
# https://github.com/seatgeek/thefuzz
#
# It returns a match ratio between the input and the choices
# This is an experimentally determined threshold that works
# for our bazel commands
THEFUZZ_MATCH_RATIO_THRESHOLD = 50
NUM_FILTERED_MATCHES_TO_SHOW = 10

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run stuff", add_help=False)

    parser.add_argument("action", choices=["build", "run", "test"])
    parser.add_argument("search_query")
    parser.add_argument("-h", "--help", action="store_true")
    parser.add_argument(
        "-p",
        "--print_command",
        action="store_true",
        help="Print the generated Bazel command",
    )
    parser.add_argument(
        "-no",
        "--no_optimized_build",
        action="store_true",
        default=False,
        help="Compile binaries without -O3 optimizations",
    )
    parser.add_argument(
        "-d",
        "--debug_build",
        action="store_true",
        help="Compile binaries with debug symbols",
    )
    parser.add_argument(
        "-ds",
        "--select_debug_binaries",
        choices=["sim", "blue", "yellow"],
        nargs="+",
        help="Select all binaries which are running separately in debug mode",
        action="store",
    )
    parser.add_argument(
        "-f",
        "--flash_robots",
        nargs="+",
        type=int,
        help="A list of space separated integers representing the robot IDs "
        "that should be flashed by the deploy_robot_software Ansible playbook",
        action="store",
    )
    parser.add_argument(
        "-pwd",
        "--pwd",
        type=str,
        help="Password used by Ansible when SSHing into the robots",
        action="store",
    )
    parser.add_argument(
        "-i",
        "--interactive",
        help="Interactively search for a bazel target",
        action="store_true",
    )
    parser.add_argument(
        "--tracy",
        help="Run the binary with the TRACY_ENABLE macro defined",
        action="store_true",
    )
    parser.add_argument(
        "-pl",
        "--platform",
        type=str,
        choices=["PI", "NANO", "LIMITED"],
        help="The platform to build Thunderloop for",
        action="store",
    )

    # These are shortcut args for commonly used arguments on our tests
    # and full_system. All other arguments are passed through as-is
    # to the underlying binary/test that is being run (unknown_args)
    parser.add_argument("-t", "--enable_thunderscope", action="store_true")
    parser.add_argument("-v", "--enable_visualizer", action="store_true")
    parser.add_argument("-s", "--stop_ai_on_start", action="store_true")
    args, unknown_args = parser.parse_known_args()

    if bool(args.flash_robots) ^ bool(args.pwd):
        print(
            "If you want to flash robots, both the robot IDs and password must be provided"
        )
        sys.exit(1)

    # If help was requested, print the help for the tbots script
    # and propagate the help to the underlying binary/test to
    # also see the arguments it supports
    if args.help:
        print(45 * "=" + " tbots.py help " + 45 * "=")
        parser.print_help()
        print(100 * "=")
        unknown_args += ["--help"]

    test_query = ["bazel", "query", "tests(//...)"]
    binary_query = ["bazel", "query", "kind(.*_binary,//...)"]
    library_query = ["bazel", "query", "kind(.*_library,//...)"]

    bazel_queries = {
        "test": [test_query],
        "run": [test_query, binary_query],
        "build": [library_query, test_query, binary_query],
    }

    # Run the appropriate bazel query and ask thefuzz to find the best matching
    # target, gauranteed to return 1 result because we set limit=1
    # Combine results of multiple queries with itertools.chain
    targets = list(
        itertools.chain.from_iterable(
            [
                run(query, stdout=PIPE).stdout.rstrip(b"\n").split(b"\n")
                for query in bazel_queries[args.action]
            ]
        )
    )
    # Create a dictionary to map target names to complete bazel targets
    target_dict = {target.split(b":")[-1]: target for target in targets}

    # Use thefuzz to find the best matching target name
    most_similar_target_name, confidence = process.extract(
        args.search_query, list(target_dict.keys()), limit=1
    )[0]
    target = str(target_dict[most_similar_target_name], encoding="utf-8")

    print("Found target {} with confidence {}".format(target, confidence))

    if args.interactive or confidence < THEFUZZ_MATCH_RATIO_THRESHOLD:
        filtered_targets = process.extract(
            args.search_query,
            list(target_dict.keys()),
            limit=NUM_FILTERED_MATCHES_TO_SHOW,
        )
        targets = [
            target_dict[filtered_target_name[0]]
            for filtered_target_name in filtered_targets
        ]
        target = str(iterfzf.iterfzf(iter(targets)), encoding="utf-8")
        print("User selected {}".format(target))

    command = ["bazel", args.action, target]

    # Trigger a debug build
    if args.debug_build or args.select_debug_binaries:
        command += ["-c", "dbg"]

    # Trigger an optimized build by default. Note that Thunderloop should always be
    # compiled with optimizations for best performance
    if not args.no_optimized_build or args.flash_robots:
        command += ["--copt=-O3"]

    # Used for when flashing Jetsons
    if args.flash_robots:
        command += ["--platforms=//cc_toolchain:robot"]

    # Select debug binaries to run
    if args.select_debug_binaries:
        if "sim" in args.select_debug_binaries:
            unknown_args += ["--debug_simulator"]
        if "blue" in args.select_debug_binaries:
            unknown_args += ["--debug_blue_full_system"]
        if "yellow" in args.select_debug_binaries:
            unknown_args += ["--debug_yellow_full_system"]

    # To run the Tracy profile, enable the TRACY_ENABLE macro
    if args.tracy:
        command += ["--cxxopt=-DTRACY_ENABLE"]

    if args.platform:
        command += ["--//software/embedded:host_platform=" + args.platform]

    # Don't cache test results
    if args.action in "test":
        command += ["--cache_test_results=false"]
    if args.action in "run":
        command += ["--"]

    bazel_arguments = unknown_args
    if args.stop_ai_on_start:
        bazel_arguments += ["--stop_ai_on_start"]
    if args.enable_visualizer:
        bazel_arguments += ["--enable_visualizer"]
    if args.enable_thunderscope:
        bazel_arguments += ["--enable_thunderscope"]
    if args.flash_robots:
        if not args.platform:
            print("No platform specified! Make sure to set the --platform argument.")
            sys.exit(1)
        bazel_arguments += ["-pb deploy_robot_software.yml"]
        bazel_arguments += ["--hosts"]
        platform_ip = "0" if args.platform == "NANO" else "6"
        bazel_arguments += [f"192.168.{platform_ip}.20{id}" for id in args.flash_robots]
        bazel_arguments += ["-pwd", args.pwd]

    if args.action in "test":
        command += ['--test_arg="' + arg + '"' for arg in bazel_arguments]

        if (
            "--debug_blue_full_system" in unknown_args
            or "--debug_yellow_full_system" in unknown_args
            or "--debug_simulator" in unknown_args
        ):
            print(
                "Do not run simulated pytests as a test when debugging, use ./tbots.py -d run instead"
            )
            sys.exit(1)

    else:
        command += bazel_arguments

    # If the user requested a command dump, just print the command to run
    if args.print_command:
        print(" ".join(command))

    # Otherwise, run the command! We use os.system here because we don't
    # care about the output and subprocess doesn't seem to run qt for somereason
    else:
        print(" ".join(command))
        code = os.system(" ".join(command))
        # propagate exit code
        sys.exit(1 if code != 0 else 0)
