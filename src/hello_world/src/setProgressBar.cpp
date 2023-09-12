#include <iostream>
#include "hello_world/progress_bar.hpp"

using namespace std;

ProgressBar *setupProgressBarPartial(const char *showText, int max)
{
    ProgressBar *bar = new ProgressBar(max, showText);
    if (max < 100)
        bar->SetFrequencyUpdate(max);
    else
        bar->SetFrequencyUpdate(100);
    bar->SetStyle("|", "-");

    return bar;
}

ProgressBar *setupProgressBarMain(const char *showText, int max)
{
    ProgressBar *bar = new ProgressBar(max, showText);
    if (max < 100)
        bar->SetFrequencyUpdate(max);
    else
        bar->SetFrequencyUpdate(100);
    bar->SetStyle("\u2588", "-");

    return bar;
}