import subprocess
import re
import os
import sys
import datetime
import csv

# --- Configuration ---
# Assuming execution from the root directory: python3 Scripts/run_benchmarks.py
BINARY_PATH = "./cmake-build-release/ULTRA" 

# Base directories
RESULTS_DIR = "./Results"
METRICS_DIR = os.path.join(RESULTS_DIR, "metrics")
LOGS_DIR = os.path.join(RESULTS_DIR, "logs")

# Ensure directories exist
os.makedirs(METRICS_DIR, exist_ok=True)
os.makedirs(LOGS_DIR, exist_ok=True)

# Generate timestamp for this run
TIMESTAMP = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

# Output files
RESULTS_TXT = os.path.join(METRICS_DIR, f"benchmark_results_{TIMESTAMP}.txt")
RESULTS_CSV = os.path.join(METRICS_DIR, f"benchmark_results_{TIMESTAMP}.csv")
FULL_LOG_FILE = os.path.join(LOGS_DIR, f"full_run_log_{TIMESTAMP}.txt")

# Define the benchmarks
# Structure: (Network, Configuration, Algorithm, Command_Args)
benchmarks = [
    # --- Switzerland Core-CH Comparison ---
    ("Switzerland", "CoreCH", "MR (Baseline)", 
     ["compareMRwithTDStatefulCoreCH", "./Networks/Switzerland/Walking/Full/raptor.binary", "./Networks/Switzerland/Walking/Full/intermediate.binary", "./Networks/Switzerland/Walking/Contracted/ch", "1000"]),
    
    ("Switzerland", "CoreCH", "TD-Dijkstra", 
     ["compareMRwithTDStatefulCoreCH", "./Networks/Switzerland/Walking/Full/raptor.binary", "./Networks/Switzerland/Walking/Full/intermediate.binary", "./Networks/Switzerland/Walking/Contracted/ch", "1000"]),
    
    ("Switzerland", "CoreCH", "MR (Pruning)", 
     ["RunDijkstraRAPTORQueries", "./Networks/Switzerland/Walking/Full/raptor.binary", "./Networks/Switzerland/Walking/Contracted/ch", "1000", "1"]),

    # --- Switzerland No-CH Comparison ---
    ("Switzerland", "No CH", "MR (Baseline)", 
     ["compareMRwithTDStatefulNoCH", "./Networks/Switzerland/Walking/Full/raptor.binary", "./Networks/Switzerland/Walking/Full/intermediate.binary", "1000"]),

    ("Switzerland", "No CH", "TD-Dijkstra", 
     ["compareMRwithTDStatefulNoCH", "./Networks/Switzerland/Walking/Full/raptor.binary", "./Networks/Switzerland/Walking/Full/intermediate.binary", "1000"]),

    ("Switzerland", "No CH", "MR (Pruning)", 
     ["RunDijkstraRAPTORQueriesNoCH", "./Networks/Switzerland/Walking/Full/raptor.binary", "1000", "1"]),

    # --- London Core-CH Comparison ---
    ("London", "CoreCH", "MR (Baseline)", 
     ["compareMRwithTDStatefulCoreCH", "./Networks/London/Walking/Full/raptor.binary", "./Networks/London/Walking/Full/intermediate.binary", "./Networks/London/Walking/Contracted/ch", "1000"]),

    ("London", "CoreCH", "TD-Dijkstra", 
     ["compareMRwithTDStatefulCoreCH", "./Networks/London/Walking/Full/raptor.binary", "./Networks/London/Walking/Full/intermediate.binary", "./Networks/London/Walking/Contracted/ch", "1000"]),

    ("London", "CoreCH", "MR (Pruning)", 
     ["RunDijkstraRAPTORQueries", "./Networks/London/Walking/Full/raptor.binary", "./Networks/London/Walking/Contracted/ch", "1000", "1"]),

    # --- London No-CH Comparison ---
    ("London", "No CH", "MR (Baseline)", 
     ["compareMRwithTDStatefulNoCH", "./Networks/London/Walking/Full/raptor.binary", "./Networks/London/Walking/Full/intermediate.binary", "1000"]),

    ("London", "No CH", "TD-Dijkstra", 
     ["compareMRwithTDStatefulNoCH", "./Networks/London/Walking/Full/raptor.binary", "./Networks/London/Walking/Full/intermediate.binary", "1000"]),

    ("London", "No CH", "MR (Pruning)", 
     ["RunDijkstraRAPTORQueriesNoCH", "./Networks/London/Walking/Full/raptor.binary", "1000", "1"]),
]

# --- Helper Functions ---

def log_to_file(message):
    """Writes a message to the full log file."""
    with open(FULL_LOG_FILE, "a") as f:
        f.write(message + "\n")

def run_interactive_command(command_args):
    """
    Runs a command interactively, printing stdout/stderr in real-time,
    logging it, and returning the full captured stdout for parsing.
    """
    input_str = " ".join(command_args) + "\n"
    cmd_display = f"Running: {' '.join(command_args)}"
    
    print(f"\n{'-'*60}")
    print(cmd_display)
    print(f"{'-'*60}")
    
    log_to_file(f"\n{'-'*60}\n{cmd_display}\n{'-'*60}")

    captured_output = []

    try:
        process = subprocess.Popen(
            [BINARY_PATH],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, # Merge stderr into stdout
            text=True,
            bufsize=1 # Line buffered
        )

        # Send input
        process.stdin.write(input_str)
        process.stdin.close()

        # Read output line by line in real-time
        for line in process.stdout:
            sys.stdout.write(line) # Print to console
            log_to_file(line.rstrip()) # Log to file
            captured_output.append(line)
        
        process.wait()
        
        if process.returncode != 0:
            err_msg = f"Error: Process exited with code {process.returncode}"
            print(err_msg)
            log_to_file(err_msg)
            return ""

        return "".join(captured_output)

    except Exception as e:
        err_msg = f"Exception running command: {e}"
        print(err_msg)
        log_to_file(err_msg)
        return ""

def extract_time(output, algorithm_name, is_comparison):
    """Parses the execution time from the command output."""
    if not output:
        return "Error"

    time_ms = "N/A"
    
    if is_comparison:
        # Flexible matching for header lines
        mr_block_start = output.find("--- Statistics MR")
        
        # Try finding TD stats with or without "(stateful)"
        td_block_start = output.find("--- Statistics TD-Dijkstra")
        
        target_block = ""
        if "MR" in algorithm_name:
            if mr_block_start != -1:
                # If TD block exists, stop there, otherwise go to end
                end_idx = td_block_start if td_block_start != -1 else len(output)
                target_block = output[mr_block_start:end_idx]
        elif "TD-Dijkstra" in algorithm_name:
            if td_block_start != -1:
                target_block = output[td_block_start:]
        
        # Regex for time
        match = re.search(r"Total time\s*:\s*([\d\.]+)\s*ms", target_block)
        if match:
            time_ms = match.group(1) + " ms"
        else:
            match_us = re.search(r"Total time\s*:\s*([\d\.]+)\s*us", target_block)
            if match_us:
                time_ms = f"{float(match_us.group(1))/1000.0:.2f} ms"
    else:
        # Single run
        match = re.search(r"Total time\s*:\s*([\d\.]+)\s*ms", output)
        if match:
            time_ms = match.group(1) + " ms"
        else:
            match = re.search(r"Total time\s*:\s*([\d\.]+)\s*us", output)
            if match:
                time_ms = f"{float(match.group(1))/1000.0:.2f} ms"
                
    return time_ms

# --- Main Execution Loop ---

results = []
comparison_cache = {} # Key: tuple(cmd_args), Value: full_output

print(f"Benchmarks started at {TIMESTAMP}")
print(f"Logging to: {FULL_LOG_FILE}")

for net, config, algo, cmd_args in benchmarks:
    cmd_tuple = tuple(cmd_args)
    is_comparison = "compare" in cmd_args[0]
    
    output = ""
    
    if is_comparison:
        # Check cache
        if cmd_tuple in comparison_cache:
            print(f"\n[Cache Hit] Using previous run for {algo}...")
            output = comparison_cache[cmd_tuple]
        else:
            output = run_interactive_command(cmd_args)
            comparison_cache[cmd_tuple] = output
    else:
        # Always run non-comparison commands (no caching logic needed here for simple structure)
        output = run_interactive_command(cmd_args)

    time_str = extract_time(output, algo, is_comparison)
    results.append((net, config, algo, time_str))


# --- Generate Outputs ---

# 1. CSV Output
with open(RESULTS_CSV, "w", newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["Network", "Configuration", "Algorithm", "Avg Time"])
    writer.writerows(results)

print(f"\nCSV results written to: {RESULTS_CSV}")

# 2. Markdown Table (TXT) - Properly Formatted
headers = ["Network", "Configuration", "Algorithm", "Avg Time"]
col_widths = [len(h) for h in headers]

# First pass to determine max width
for r in results:
    for i, val in enumerate(r):
        col_widths[i] = max(col_widths[i], len(str(val)))

# Formatting function
def format_row(row_data):
    return "| " + " | ".join(f"{str(val):<{width}}" for val, width in zip(row_data, col_widths)) + " |"

# Header
header_row = format_row(headers)
separator_row = "|-" + "-|-".join("-" * width for width in col_widths) + "-|"

markdown_lines = [header_row, separator_row]

last_net = ""
last_config = ""

for r in results:
    current_net, current_config = r[0], r[1]
    
    # Aesthetic separator (empty row with pipes)
    if last_net and (current_net != last_net or current_config != last_config):
        # We only add a separator if it's not the very first row
        empty_row = "| " + " | ".join(" " * width for width in col_widths) + " |"
        markdown_lines.append(empty_row)
             
    markdown_lines.append(format_row(r))
    last_net = current_net
    last_config = current_config

final_md = "\n".join(markdown_lines)

print("\n" + final_md)

with open(RESULTS_TXT, "w") as f:
    f.write(final_md)

print(f"Markdown results written to: {RESULTS_TXT}")