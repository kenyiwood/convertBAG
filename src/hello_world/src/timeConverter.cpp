#include <iostream>
#include <string>
#include <ctime>

using namespace std;
int dayOfTheWeek(int year, int month, int day)
{
    // Mon Tue Wed Thu Fri Sat Sun
    struct tm t;
    time_t t_of_day;
    t.tm_year = year - 1900;
    t.tm_mon = month - 1;
    t.tm_mday = day;
    t.tm_hour = 12;
    t.tm_min = 0;
    t.tm_sec = 1;
    t.tm_isdst = 0;

    t_of_day = mktime(&t);
    // cout << ctime(&t_of_day) << endl;
    string dd = "";
    dd += ctime(&t_of_day)[0];
    dd += ctime(&t_of_day)[1];
    dd += ctime(&t_of_day)[2];

    int returnValue = 0;
    if (dd.compare("Sun") == 0)
        returnValue = 0;
    else if (dd.compare("Mon") == 0)
        returnValue = 1;
    else if (dd.compare("Tue") == 0)
        returnValue = 2;
    else if (dd.compare("Wed") == 0)
        returnValue = 3;
    else if (dd.compare("Thu") == 0)
        returnValue = 4;
    else if (dd.compare("Fri") == 0)
        returnValue = 5;
    else if (dd.compare("Sat") == 0)
        returnValue = 6;
    return returnValue;
}

double secondsDifference(int year0, int month0, int day0, int year1, int month1, int day1)
{
    struct tm t;
    time_t t_of_day;
    t.tm_year = year0 - 1900;
    t.tm_mon = month0 - 1;
    t.tm_mday = day0;
    t.tm_hour = 12;
    t.tm_min = 0;
    t.tm_sec = 1;
    t.tm_isdst = 0;
    t_of_day = mktime(&t);

    struct tm t2;
    time_t t_of_day2;
    t2.tm_year = year1 - 1900;
    t2.tm_mon = month1 - 1;
    t2.tm_mday = day1;
    t2.tm_hour = 12;
    t2.tm_min = 0;
    t2.tm_sec = 1;
    t2.tm_isdst = 0;

    t_of_day2 = mktime(&t2);

    return difftime(t_of_day2, t_of_day);
}

int secondsDifferenceBetweenIEAndROS(int year, int month, int day)
{
    double secondsFrom1970 = secondsDifference(1970, 1, 1, year, month, day);
    int secondsFromThisSunday = dayOfTheWeek(year, month, day);
    int totalSecondsBetweenIEAndROS = secondsFrom1970 - secondsFromThisSunday * 86400 - 18;
    return totalSecondsBetweenIEAndROS;
}