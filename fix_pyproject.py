import toml
import re

with open("pyproject.toml", "r") as f:
    content = f.read()

# Using regex to insert it safely if not present
if "[project.optional-dependencies]" not in content:
    content = content.replace("[project.scripts]", "[project.optional-dependencies]\ncan = [\"python-can>=4.3\"]\n\n[project.scripts]")
    with open("pyproject.toml", "w") as f:
        f.write(content)
