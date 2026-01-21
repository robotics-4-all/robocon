#!/usr/bin/env python3

import os
import subprocess
import sys
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich.progress import Progress, SpinnerColumn, BarColumn, TextColumn, TimeElapsedColumn

def validate_examples():
    console = Console()
    examples_dir = os.path.join(os.getcwd(), 'examples')
    
    if not os.path.exists(examples_dir):
        console.print(f"[bold red]Error:[/bold red] Examples directory '{examples_dir}' not found.")
        sys.exit(1)

    rbr_files = [f for f in os.listdir(examples_dir) if f.endswith('.rbr')]
    rbr_files.sort()

    if not rbr_files:
        console.print("[yellow]No .rbr files found in examples directory.[/yellow]")
        return

    table = Table(title="Robocon Examples Validation")
    table.add_column("Example File", style="cyan")
    table.add_column("Status", justify="center")
    table.add_column("Details", style="dim")

    success_count = 0
    fail_count = 0

    console.print(Panel(f"[bold blue]Validating {len(rbr_files)} examples...[/bold blue]"))

    # Try to find robocon executable
    robocon_bin = 'robocon'
    # Check if we are in a venv and if robocon is there
    venv_bin = os.path.join(os.path.dirname(sys.executable), 'robocon')
    if os.path.exists(venv_bin):
        robocon_bin = venv_bin

    # Create progress bar
    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        BarColumn(),
        TextColumn("[progress.percentage]{task.percentage:>3.0f}%"),
        TimeElapsedColumn(),
        console=console
    ) as progress:
        
        task = progress.add_task("[cyan]Validating examples...", total=len(rbr_files))
        
        for filename in rbr_files:
            progress.update(task, description=f"[cyan]Validating {filename}...")
            filepath = os.path.join(examples_dir, filename)
            try:
                # Run robocon validate command
                result = subprocess.run(
                    [robocon_bin, 'validate', filepath],
                    capture_output=True,
                    text=True,
                    check=False
                )
                
                if result.returncode == 0:
                    table.add_row(filename, "[bold green]PASS[/bold green]", "Validated successfully")
                    success_count += 1
                else:
                    error_msg = result.stderr.strip().split('\n')[-1] if result.stderr else "Unknown error"
                    table.add_row(filename, "[bold red]FAIL[/bold red]", error_msg)
                    fail_count += 1
            except Exception as e:
                table.add_row(filename, "[bold red]ERROR[/bold red]", str(e))
                fail_count += 1
            
            progress.advance(task)

    console.print()
    console.print(table)

    if fail_count == 0:
        console.print(f"\n[bold green]Success![/bold green] All {success_count} examples validated successfully.")
    else:
        console.print(f"\n[bold red]Validation failed.[/bold red] {success_count} passed, {fail_count} failed.")
        sys.exit(1)

if __name__ == "__main__":
    validate_examples()
