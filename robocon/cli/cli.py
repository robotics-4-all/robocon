import click
from rich import print, pretty

from robocon.generator import GeneratorROS2, GeneratorROS
from robocon.utils import build_model

pretty.install()


@click.group()
@click.pass_context
def cli(ctx):
    ctx.ensure_object(dict)


@cli.command("validate", help="Model Validation")
@click.pass_context
@click.argument("model_path")
def validate(ctx, model_path):
    model = build_model(model_path)
    print("[*] Model validation success!!")


@cli.command("gen", help="Code generation")
@click.pass_context
@click.argument("model_path")
@click.argument("generator", required=False)
@click.option('-o', '--out-dir', required=False, type=str,
              default="gen", show_default=True)
def generate_code(ctx, model_path, generator, out_dir):
    model, imports = build_model(model_path)
    
    # Auto-detect generator from model if not specified
    if generator is None:
        if model.robot.type == 'ROS2':
            generator = 'ros2'
            print("[*] Auto-detected ROS2 from model")
        elif model.robot.type == 'ROS':
            generator = 'ros'
            print("[*] Auto-detected ROS from model")
        else:
            print("[bold red]Error:[/bold red] Could not determine ROS version from model")
            return
    
    if generator in ('ros2', 'ROS2', 'Ros2'):
        GeneratorROS2.generate(model, out_dir)
    elif generator in ('ros', 'ROS', 'Ros'):
        GeneratorROS.generate(model, out_dir)
    else:
        print(f"[bold red]Error:[/bold red] Unknown generator '{generator}'. Use 'ros' or 'ros2'")
        return
    
    print(f"[CLI] Generated Bridge: {out_dir}/")


def main():
    cli(prog_name="robocon")
