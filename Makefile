version=magiwanders/chipyard:0.3

default: help

help:
	cat README.md

build:
	docker image build -t $(version) .
	
container: 
	docker container create -it --mount type=bind,source='$(shell pwd)'/user,target=/home/user --name chipyard $(version) bash
	docker container ls -a

start:
	docker container start -i chipyard

clean:
	docker container stop chipyard
	docker container rm chipyard
	docker container ls -a

uninstall:
	docker container rm chipyard
	docker image rm $(version)
	
