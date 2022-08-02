#include <cmath>

// src: The Art of Computer Programming: Seminumerical algorithms
// also: https://stackoverflow.com/a/253874

template<typename T>
bool approximatelyEqual(T a, T b, T epsilon)
{
	return fabs(a - b) <= ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * epsilon);
}

template<typename T>
bool essentiallyEqual(T a, T b, T epsilon)
{
	return fabs(a - b) <= ( (fabs(a) > fabs(b) ? fabs(b) : fabs(a)) * epsilon);
}

template<typename T>
bool definitelyGreaterThan(T a, T b, T epsilon)
{
	return (a - b) > ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * epsilon);
}

template<typename T>
bool definitelyLessThan(T a, T b, T epsilon)
{
	return (b - a) > ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * epsilon);
}
