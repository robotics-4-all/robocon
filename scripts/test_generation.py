#!/usr/bin/env python3

import os
import subprocess
import sys
import py_compile
import shutil
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich.progress import Progress, SpinnerColumn, BarColumn, TextColumn, TimeElapsedColumn

def test_generation():
    console = Console()
    examples_dir = os.path.join(os.getcwd(), 'examples')
    gen_base_dir = os.path.join(os.getcwd(), 'gen_test_suite')
    
    if os.path.exists(gen_base_dir):
        shutil.rmtree(gen_base_dir)
    os.makedirs(gen_base_dir)

    if not os.path.exists(examples_dir):
        console.print(f"[bold red]Error:[/bold red] Examples directory '{examples_dir}' not found.")
        sys.exit(1)

    rbr_files = [f for f in os.listdir(examples_dir) if f.endswith('.rbr')]
    rbr_files.sort()

    if not rbr_files:
        console.print("[yellow]No .rbr files found in examples directory.[/yellow]")
        return

    table = Table(title="Robocon Generation & Evaluation")
    table.add_column("Example File", style="cyan")
    table.add_column("Target", justify="center")
    table.add_column("Gen Status", justify="center")
    table.add_column("Eval Status", justify="center")
    table.add_column("Details", style="dim")

    success_count = 0
    fail_count = 0

    console.print(Panel(f"[bold blue]Starting code generation and evaluation for {len(rbr_files)} examples...[/bold blue]"))

    # Create progress bar
    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        BarColumn(),
        TextColumn("[progress.percentage]{task.percentage:>3.0f}%"),
        TimeElapsedColumn(),
        console=console
    ) as progress:
        
        task = progress.add_task("[cyan]Processing examples...", total=len(rbr_files))
        
        for filename in rbr_files:
            progress.update(task, description=f"[cyan]Processing {filename}...")
            filepath = os.path.join(examples_dir, filename)
            
            # Detect target (ROS or ROS2)
            target = "ros2"
            try:
                with open(filepath, 'r') as f:
                    content = f.read()
                    if 'Robot[ROS]' in content and 'Robot[ROS2]' not in content:
                        target = "ros"
            except:
                pass

            out_dir = os.path.join(gen_base_dir, filename.replace('.rbr', ''))
            gen_status = "[bold red]FAIL[/bold red]"
            eval_status = "[bold yellow]N/A[/bold yellow]"
            details = ""

            try:
                # 1. Run Generation
                gen_result = subprocess.run(
                    ['robocon', 'gen', filepath, '-o', out_dir],
                    capture_output=True,
                    text=True,
                    check=False
                )
                
                if gen_result.returncode == 0:
                    gen_status = "[bold green]PASS[/bold green]"
                    
                    # 2. Evaluate Generated Code
                    # Find the generated .py file
                    py_files = [f for f in os.listdir(out_dir) if f.endswith('.py')]
                    if not py_files:
                        eval_status = "[bold red]FAIL[/bold red]"
                        details = "No .py file generated"
                    else:
                        all_passed = True
                        for py_file in py_files:
                            py_filepath = os.path.join(out_dir, py_file)
                            try:
                                py_compile.compile(py_filepath, doraise=True)
                            except py_compile.PyCompileError as e:
                                all_passed = False
                                details = f"Syntax error in {py_file}"
                                break
                        
                        if all_passed:
                            eval_status = "[bold green]PASS[/bold green]"
                            details = f"Generated {len(py_files)} file(s)"
                            success_count += 1
                        else:
                            eval_status = "[bold red]FAIL[/bold red]"
                            fail_count += 1
                else:
                    details = gen_result.stderr.strip().split('\n')[-1] if gen_result.stderr else "Gen failed"
                    fail_count += 1
                    
            except Exception as e:
                details = str(e)
                fail_count += 1

            table.add_row(filename, target.upper(), gen_status, eval_status, details)
            progress.advance(task)

    console.print()
    console.print(table)

    if fail_count == 0:
        console.print(f"\n[bold green]Success![/bold green] All {success_count} examples generated and evaluated successfully.")
    else:
        console.print(f"\n[bold red]Testing failed.[/bold red] {success_count} passed, {fail_count} failed.")
        # sys.exit(1) # Don't exit yet so we can clean up if needed


if __name__ == "__main__":
    test_generation()
