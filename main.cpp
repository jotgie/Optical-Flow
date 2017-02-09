#include "stdafx.h"
#include "mMinWindow.h"
#include "BGS.h"
#include <vector>
using namespace std;
using namespace cv;

#include <libgen.h>
#include <unistd.h>

enum class VideoMode {
    Normal,
    FrameByFrame
};

int main(void)
{
	Size winSize(15, 15), subPixWinSize(15, 15);
    //uzywane w mMinWindow
    //im mniejszy 3ci paramert tym wolniejsze działanie, a zmiana nie widoczna
    TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 30, 0.3);
    VideoMode mode = VideoMode::Normal;
    Mat mFrame0, mFrame1, mFrame2, frame;
	Mat* result1;
	VideoCapture cap;
    cv::Rect win1;

    char buf[1024] = {0};
    ssize_t size = readlink("/proc/self/exe", buf, sizeof(buf));
    if (size == 0 || size == sizeof(buf))
    {
        return 2;
    }
    std::string path(dirname(buf));
    std::cout << path << std::endl;

    cap.open("file://" + path + "/banan_ffmpeg.avi");

	if (!cap.isOpened()) {
		cout << "Cannot open the video file" << endl;
		return -1;
	}

	cap >> frame;

    win1 = cv::Rect(400, 200, 300, frame.rows - 200); //x,y,width, height
	mMinWindow mMinFrame0 = mMinWindow(win1, winSize, subPixWinSize, termcrit);
    //Do zbadania: 2 i 3 argument (history, varThreshold)
    BGS bgsFrame0 = BGS(win1, 300, 20, 0, win1.width, 300);

	while (true)
	{
		cap >> frame;

		if (frame.empty()) {
            //na koncy wyświetlam informache ile pojazdów zostało zareestrowanych i jakie miały rozmiary
            bgsFrame0.printVehicleInfo();
			fprintf(stderr, "End of video");
			return -1;
		}

		mFrame0 = mMinFrame0.drawVectors(frame);
		result1 = bgsFrame0.drawSquare(frame, mMinFrame0.getResultVector());
        mFrame1 = result1[0];
        mFrame2 = result1[1];

        //prostokąty
        imshow("Optical Flow", mFrame2);
        //kontury
        imshow("Optical Flow1", mFrame1);
        //vectory
        imshow("Optical Flow2", mFrame0);

        //p pauzuje i przechodzi so trybu klatka by klatka
        //koejne p wraca do normalnego trybu
        //dowolny klawisz prócz p przesuwa o 1 klatję do przodu w trybie klatka by klatka
        int key_pressed = waitKey(1);
        char key = static_cast<char>(key_pressed);
        if (key == 'q') break;
        if (mode == VideoMode::FrameByFrame)
        {
            key_pressed = waitKey(0);
            key = static_cast<char>(key_pressed);
        }
        if (key == 'p') {
            switch(mode){
                case VideoMode::Normal : mode = VideoMode::FrameByFrame; std::cout << "switch to frame by frame mode" << std::endl; break;
                case VideoMode::FrameByFrame : mode = VideoMode::Normal; std::cout << "switch to normal mode" << std::endl; break;
            }
        }

	}

	return 0;
}
