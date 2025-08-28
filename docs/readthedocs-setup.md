# Read the Docs Versioning Setup

This document explains how the Read the Docs (RTD) versioning is configured for this project to support separate documentation versions for development and releases.

## Overview

The documentation is configured to build multiple versions:

1. **Devel Version**: Built from the `devel` branch - contains the latest development documentation
2. **Release Versions**: Built from git tags - contains stable release documentation
3. **Latest Version**: Built from the `main` branch - contains the current stable documentation

## Configuration Files

### `.readthedocs.yaml`

This file configures how Read the Docs builds the documentation:

- Uses Ubuntu 22.04 with Python 3.10
- Uses MkDocs as the documentation generator  
- Installs dependencies from `requirements.txt`
- Automatically detects branches and tags for version building

### `mkdocs.yml`

The MkDocs configuration has been updated to be version-aware:

- Removed hardcoded `edit_uri` to allow RTD to automatically set the correct branch/tag for editing links
- Organized plugins in a logical order
- Maintains all existing themes and extensions

## How Versioning Works

### Automatic Version Creation

Read the Docs automatically creates versions for:

1. **Active Branches**: Any branch that has been activated in the RTD admin interface
2. **Active Tags**: Any git tag that has been activated in the RTD admin interface

### Setting Up Versions (RTD Admin Required)

To enable the versioning described in the requirements, a project administrator needs to:

1. **Access RTD Admin Panel**: Go to readthedocs.io project settings
2. **Activate Devel Branch**: 
   - Go to "Versions" section
   - Find the `devel` branch and activate it
   - Set it to be publicly visible
   - Optionally set a custom name like "Development"

3. **Configure Tag Versioning**:
   - Ensure "Build tags" is enabled in project settings
   - Tags will automatically become versions when created
   - Each tag creates a separate documentation version

4. **Set Default Version**:
   - Set the default version (usually "latest" pointing to main branch)
   - Configure version ordering if needed

### Creating Release Versions

To create a new release documentation version:

1. **Create and Push a Git Tag**:
   ```bash
   git tag -a v1.0.0 -m "Release version 1.0.0"
   git push origin v1.0.0
   ```

2. **RTD Auto-Detection**: Read the Docs will automatically:
   - Detect the new tag
   - Create a new version entry
   - Build documentation for that tag
   - Make it available at `https://your-project.readthedocs.io/en/v1.0.0/`

### Accessing Different Versions

Once configured, users can access different versions:

- **Latest (main)**: `https://swarm-nxt.readthedocs.io/en/latest/`
- **Development**: `https://swarm-nxt.readthedocs.io/en/devel/`  
- **Release v1.0.0**: `https://swarm-nxt.readthedocs.io/en/v1.0.0/`

## Workflow Integration

### GitHub Actions Compatibility

The existing GitHub Actions workflows in `.github/workflows/check_docs.yaml` remain compatible:

- Continues to build and test documentation on pushes
- Works alongside RTD builds (they're independent)
- Still deploys to GitHub Pages for the main branch

### Development Workflow

1. **Development Changes**: 
   - Make documentation changes in feature branches
   - Merge to `devel` branch for development documentation
   - RTD automatically rebuilds `devel` version

2. **Release Process**:
   - Merge `devel` to `main` when ready for release
   - Create a git tag for the release
   - RTD automatically creates a new release version
   - Latest version (main) gets updated

## Benefits

1. **Separate Development Docs**: Teams can preview documentation changes before they're released
2. **Version History**: Users can access documentation for specific releases they're using
3. **Automatic Management**: Once configured, versions are created automatically with git workflow
4. **Edit Links**: Edit buttons automatically point to the correct branch/tag in GitHub
5. **Search**: RTD provides version-specific search functionality

## Migration Notes

- No changes needed to existing documentation content
- Edit links will now work correctly for all versions  
- All existing functionality is preserved
- The setup is backward compatible with current workflows

## Troubleshooting

### Version Not Appearing

If a branch or tag doesn't appear as a version:
1. Check that it's activated in RTD admin panel
2. Verify the branch/tag exists and has documentation files
3. Check RTD build logs for any errors

### Build Failures

If builds fail for specific versions:
1. Check that `requirements.txt` is compatible with the RTD environment
2. Verify all documentation files are present in that branch/tag
3. Review RTD build logs for specific error messages

### Edit Links Not Working

If edit links point to wrong branch:
1. Ensure `edit_uri` is not hardcoded in `mkdocs.yml`
2. Let RTD automatically determine the correct edit URI
3. Verify the repository URL is correct in mkdocs configuration