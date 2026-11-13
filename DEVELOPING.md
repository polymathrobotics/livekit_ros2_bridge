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
`main` with a GitHub release, and the notes on the
[Releases page](https://github.com/polymathrobotics/livekit_ros2_bridge/releases) are the
changelog.

A change is breaking when it can stop a deployed client or parameter file from working. That
includes:

- an incompatible change to the client protocol in [docs/protocol.md](docs/protocol.md), which also
  bumps the protocol version `v` there
- a renamed or removed ROS parameter, or a changed default
- a renamed topic, service, or track that the bridge serves

Before 1.0.0, breaking changes and new features bump the minor version, and fixes bump the patch
version.

Only release PRs change the version in `package.xml`. Feature and fix PRs leave it alone, and
describe their user-facing effect in the PR description, saying clearly when it is breaking.

1. Open a PR that sets the new version in `package.xml`.
2. Once it merges, draft a GitHub release with tag `vX.Y.Z` targeting the commit the PR landed as,
   and generate its notes.
3. Above the generated PR list, write the user-facing changes under Breaking, Added, Changed, and
   Fixed, leaving out empty headings, with a migration note for each breaking change. Then publish;
   publishing creates the tag.
