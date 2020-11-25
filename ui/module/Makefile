ACCOUNT=pi@10.0.0.147

PPATH=projects/ui

LDFLAGS =  -g -Wall -lstdc++

%.o : %.C
	g++ -g -Wall -std=c++11 -I/opt/vc/include -DRASPPI -DNDEBUG $(CFLAGS) -fno-rtti -fPIC -c $< -o $@

all:: build

build: clean
	rsync -a .. ${ACCOUNT}:${PPATH}
	ssh ${ACCOUNT} "cd ${PPATH}/module ; export CFLAGS=-DRASPBERRY ; make rasp"

shutdown:
	ssh ${ACCOUNT} "sudo shutdown -h now"

win:  module.o
	g++ -shared -o libhardwareModule.so $^

rasp: module.o pca9685.o tof.o
	g++ -shared -L/usr/local/lib -o libhardwareModule.so $^ -lbcm2835

clean:
	rm -f *.o *.so
