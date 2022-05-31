# streamlight
A containerized python application that uses the groundlight sdk to
process frames from a video stream

## build

The docker image uses the groundlight python sdk. We need to make aws
credentials available to the build process so it can authenticate to
aws codeartifact because the python package is private as of this
writing. Set the following env variables with real credentials

``` shell
export AWS_ACCESS_KEY_ID=AKIAIOSFODNN7EXAMPLE
export AWS_SECRET_ACCESS_KEY=wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY
```
Then you can build the docker image exposing these envs as build args.

``` shell
docker build -t stream:local --build-arg AWS_ACCESS_KEY_ID --build-arg AWS_SECRET_ACCESS .
```

## run
Now you can run it
