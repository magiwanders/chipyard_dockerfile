# [Chipyard](https://chipyard.readthedocs.io/en/stable/) Dockerfile
Dockerfile for the [Chipyard SoC development framework docker image](https://hub.docker.com/r/magiwanders/chipyard). 

Build an image:

> :warning: **This builds the image with its current DockerHub name and version `magiwanders/chipyard:x.x`. If you want to publish your own fork modify the first line of the Makefile.**

```
make build
```

Create non-volatile container and run it:

```
make container
make start
```

Remove container:
```make clean```

Remove container and image:
```make uninstall```