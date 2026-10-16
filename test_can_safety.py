import tomli
import sys
with open('pyproject.toml', 'rb') as f:
    d = tomli.load(f)
    print("Extras:", d.get("project", {}).get("optional-dependencies", {}))
