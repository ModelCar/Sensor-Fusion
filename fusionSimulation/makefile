fusion : fusion.o obstacle.o ../kstatics.o
	g++ -g -o $@ $+ -Wno-deprecated -lkalman -L../ 

fusion.o : fusion.cpp ../kalman/obstacle.h ../kalman/ekfilter.hpp ../kalman/ekfilter_impl.hpp
	g++ -g -c -o $@ $< -I../kalman

obstacle.o : obstacle.cpp ../kalman/obstacle.h ../kalman/ekfilter.hpp ../kalman/ekfilter_impl.hpp
	g++ -g -c -o $@ $< -I../kalman

.PHONY : clean

clean :
	rm -f example;
	rm -f *.o
