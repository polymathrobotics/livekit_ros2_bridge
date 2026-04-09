# LiveKit ROS2 Bridge

Generic bridge for connecting ROS2 pub/sub into LiveKit.

## Local development

The local workflow is intentionally small:

- `just build`
- `just test`
- `just format`
- `just clean`

`just build` and `just test` auto-create the Docker dev image, container, and
named volumes for the active `ROS_DISTRO`. The default distro is `humble`.

Extra `colcon` arguments go after `--`:

- `just build -- --cmake-force-configure`
- `just test -- --ctest-args -R test_runtime`

`just format` runs `pre-commit run --all-files` on the host. `just clean`
removes the local dev container and its named volumes for the current
`ROS_DISTRO` or `DEV_RESOURCE_PREFIX`.

Prerequisites:

- `docker` with `buildx`
- `just`
- `pre-commit` for `just format`

## Docker image builds

Rare image work no longer goes through `just`. Use `docker buildx bake`
directly with [`docker/bake.hcl`](docker/bake.hcl), for example:

- `docker buildx bake --file docker/bake.hcl --print all`
- `docker buildx bake --file docker/bake.hcl --load dev-humble`
- `docker buildx bake --file docker/bake.hcl runtime-all`

If you rebuild the dev image manually and want a fresh local container, run
`just clean` before the next `just build` or `just test`.

## License

Apache License 2.0
