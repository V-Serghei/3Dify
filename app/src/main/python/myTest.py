#helloworld.py
import pkg_resources


def get_text(name : str):
    return "Привет, " + name

def list_installed_packages():
    packages = [pkg.project_name for pkg in pkg_resources.working_set]
    return ', '.join(packages)

