# Developing

`livekit_ros2_bridge` is a standard ROS 2 package. After sourcing the target ROS 2 environment and
workspace, use normal `colcon` commands from the workspace root:

```bash
colcon build --packages-up-to livekit_ros2_bridge
colcon test --packages-select livekit_ros2_bridge
colcon test-result --verbose
```

The build infers the LiveKit SDK artifact from `ROS_DISTRO` when `LIVEKIT_SDK_DISTRO` is unset, so
make sure the sourced environment matches the distro you intend to build.

## Formatting and linting

Formatting and linting come from the shared
[Polymath code standard](https://github.com/polymathrobotics/polymath_code_standard) pre-commit
hooks, which bundle their own clang-format, cpplint, cmakelint, yamlfix, and pymarkdown settings.
Install [pre-commit](https://pre-commit.com), for example with
[uv](https://github.com/astral-sh/uv), then enable the hooks in your clone:

```bash
uv tool install --with pre-commit-uv pre-commit
pre-commit install
```

To check the whole tree rather than only staged files, run `pre-commit run --all-files`.

## Releasing

The bridge follows [semantic versioning](https://semver.org). A release is a `vX.Y.Z` tag on
`main` with a GitHub release, and the release notes are the changelog. `polymath_core` pins the
bridge to a release tag in `repos-jazzy.yaml`.

A change is breaking when it can stop a deployed client or launch file from working: the client
protocol in [docs/protocol.md](docs/protocol.md), the ROS parameters, or the names of the topics,
services, and tracks the bridge serves. Before 1.0.0, breaking changes and new features bump the
minor version and fixes bump the patch version.

1. Open a PR that sets the new version in `package.xml`.
2. Once it merges, tag the merge commit and create the release:

   ```bash
   gh release create vX.Y.Z --target <merge commit> --generate-notes
   ```

3. Edit the release notes: above the generated PR list, describe the user-facing changes, with any
   breaking change first.
4. Bump the bridge's `version` in `polymath_core`'s `repos-jazzy.yaml` to the new tag.
