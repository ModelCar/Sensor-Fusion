videosimulation : videosimulation.o obstacle.o ../kstatics.o
	g++ -g -std=c++11 -o $@ $+ -Wno-deprecated -lkalman -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lopencv_objdetect -lm -lz -lpthread -lavformat -lavcodec -lavutil -L../ 

videosimulation.o : videosimulation.cpp ../kalman/obstacle.h ../kalman/ekfilter.hpp ../kalman/ekfilter_impl.hpp
	g++ -g -c -std=c++11 -o $@ $< -I../kalman

obstacle.o : obstacle.cpp ../kalman/obstacle.h ../kalman/ekfilter.hpp ../kalman/ekfilter_impl.hpp
	g++ -g -c -std=c++11 -o $@ $< -I../kalman

.PHONY : clean

clean :
	rm -f example;
	rm -f *.o	
