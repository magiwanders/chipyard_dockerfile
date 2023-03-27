version=magiwanders/chipyard:0.1

all:
	docker image build -t $(version) .
	
container: 
	docker container create -it --name chipyard $(version) bash
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
	
