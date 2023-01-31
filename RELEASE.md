This project is meant to be public and available to potential
customers to get started using our API, so we publish the image to
dockerhub.com instead of AWS like our internal images. We document the
release process here instead of the README.md so the latter only
contains info that can be used on dockerhub and pypi for potential
users.

# steps
We follow a standard process of build, tag, auth, push where the only
external dependency is dockerhub

## build
from the root of this repository
``` shell
docker build -t stream:local .
```
## tag
we need to tag the locally build image to include the full url of the
registry to which it will be pushed

``` shell
docker tag stream:local groundlight/stream:test
```

replace `test` with the actual release version. See [recent
tags](https://hub.docker.com/repository/docker/groundlight/stream/tags?page=1&ordering=last_updated)
and pick an appropriate [semantic version](https://semver.org/)

## authenticate

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
## push
You need to be part of the [devs
team](https://hub.docker.com/orgs/groundlight/teams/devs/members) in
the groundlight organization to be able to push images to the
streamlight repository. Ask to be added in the #engineering channel on
slack. Note: this is a one time step and won't be necessary once we
push images from a github action instead of this manual process.

push using the actual tag created previous instead of just `test`
``` shell
docker push groundlight/stream:test
```
you can verify that the new image was pushed on the
[tags](https://hub.docker.com/repository/docker/groundlight/stream/tags?page=1&ordering=last_updated)
page
