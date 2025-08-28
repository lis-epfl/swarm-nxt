#!/bin/bash

# Test script to validate Read the Docs configuration
# This script tests that the documentation builds correctly
# and validates the RTD configuration files

set -e

echo "=== Testing Read the Docs Configuration ==="
echo

# Check required files exist
echo "1. Checking configuration files..."
if [ ! -f ".readthedocs.yaml" ]; then
    echo "❌ ERROR: .readthedocs.yaml not found"
    exit 1
fi

if [ ! -f "mkdocs.yml" ]; then
    echo "❌ ERROR: mkdocs.yml not found"
    exit 1
fi

if [ ! -f "requirements.txt" ]; then
    echo "❌ ERROR: requirements.txt not found"
    exit 1
fi

echo "✅ Configuration files found"

# Test YAML syntax
echo
echo "2. Validating YAML syntax..."
python3 -c "import yaml; yaml.safe_load(open('.readthedocs.yaml'))" 2>/dev/null && echo "✅ .readthedocs.yaml syntax valid" || (echo "❌ ERROR: .readthedocs.yaml syntax invalid" && exit 1)
python3 -c "import yaml; yaml.safe_load(open('mkdocs.yml'))" 2>/dev/null && echo "✅ mkdocs.yml syntax valid" || (echo "❌ ERROR: mkdocs.yml syntax invalid" && exit 1)

# Test Python dependencies installation
echo
echo "3. Testing Python dependencies..."
pip install -q -r requirements.txt && echo "✅ Dependencies installed successfully" || (echo "❌ ERROR: Failed to install dependencies" && exit 1)

# Test documentation build
echo
echo "4. Testing documentation build..."
mkdocs build --clean --strict >/dev/null 2>&1 && echo "✅ Documentation builds successfully" || (echo "❌ ERROR: Documentation build failed" && exit 1)

# Check that all nav pages exist
echo
echo "5. Checking navigation pages..."
python3 -c "
import yaml
import os
config = yaml.safe_load(open('mkdocs.yml'))
nav = config.get('nav', [])
missing = []
for item in nav:
    if isinstance(item, dict):
        for title, path in item.items():
            if not path.startswith('http') and not os.path.exists(f'docs/{path}'):
                missing.append(f'{title}: {path}')
            
if missing:
    print('❌ ERROR: Missing navigation pages:')
    for page in missing:
        print(f'   - {page}')
    exit(1)
else:
    print('✅ All navigation pages exist')
" && echo "✅ Navigation validated" || exit 1

# Check RTD configuration
echo
echo "6. Validating RTD configuration..."
python3 -c "
import yaml
rtd_config = yaml.safe_load(open('.readthedocs.yaml'))

# Check required sections
required = ['version', 'build', 'mkdocs', 'python']
missing = [section for section in required if section not in rtd_config]

if missing:
    print(f'❌ ERROR: Missing RTD config sections: {missing}')
    exit(1)

# Check python dependencies
if 'install' not in rtd_config['python']:
    print('❌ ERROR: python.install not configured in RTD config')
    exit(1)

print('✅ RTD configuration valid')
"

echo
echo "=== All tests passed! ==="
echo "✅ Ready for Read the Docs versioning:"
echo "   - Devel branch builds will create a 'devel' version"
echo "   - Tagged releases will create numbered versions" 
echo "   - Main branch builds will update the 'latest' version"
echo
echo "Next steps:"
echo "1. Activate 'devel' branch in RTD admin panel"
echo "2. Enable tag building in RTD project settings"
echo "3. Create git tags for releases to trigger version builds"