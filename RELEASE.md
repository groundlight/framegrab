# Multi-arch Build
We follow a standard process of build, tag, auth, push where the only
external dependency is dockerhub

```
docker login
docker buildx create --name multibuilder --driver docker-container --bootstrap
docker buildx use multibuilder
docker buildx build \
    --platform linux/arm64,linux/amd64 \
    --push \
    -t groundlight/stream:VERSION .
```

replacing `VERSION` with the current version number, like `0.1.1`.  DO NOT include a `v` in the version.

To update the `:latest` tag, run the same command without the version specified:

```
docker buildx build \
    --platform linux/arm64,linux/amd64 \
    --push \
    -t groundlight/stream .
```

(We should consider adding `linux/arm/v7` as another supported arch.)

## Checking the build

See the page on dockerhub: [https://hub.docker.com/r/groundlight/stream/tags](https://hub.docker.com/r/groundlight/stream/tags)

## authenticating to dockerhub

``` shell
docker login
```

This gets your credentials and authenticate with `docker.io` by
default. It may not require that you type credential again if you have
authenticated in the past and your credentials were stored locally
(e.g. macOS keychain). On some systems you may need `sudo`. [Learn
more](https://docs.docker.com/engine/reference/commandline/login/#privileged-user-requirement)

You can verify that your docker daemon is authenticated with the correct registry

``` shell
$ docker info
Client:
 ...
Server:
 ...
 Registry: https://index.docker.io/v1/
 ...
```
