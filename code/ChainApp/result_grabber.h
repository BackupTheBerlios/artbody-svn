#ifndef _RESULTS_GRABBER_H_
#define _RESULTS_GRABBER_H_

#include <list>

namespace rnd {
    class Renderer;
}

namespace res {

    template <class T>
    class ResultsGrabber {
    public:
        explicit ResultsGrabber();
        virtual ~ResultsGrabber();

        bool addResult(T value);

        bool renderResults(rnd::Renderer* renderer);

        bool flushResultsToFile(const char* fileName);
    private:
        std::list<T> _results;
    };

    template<class T> ResultsGrabber<T>::ResultsGrabber()
    {
        ;
    }

    template<class T> ResultsGrabber<T>::~ResultsGrabber()
    {
        _results.clear();
    }

    template<class T> bool ResultsGrabber<T>::addResult(T value)
    {
        _results.push_back(value);
        return true;
    }

    template<class T> bool ResultsGrabber<T>::renderResults(rnd::Renderer* renderer)
    {
        std::list<T>::iterator iter;

        for (iter = _results.begin(); iter != _results.end(); iter++) {
            T& value = *iter;        
        }

        return true;
    }

    template<class T> bool ResultsGrabber<T>::flushResultsToFile(const char * fileName)
    {
        FILE * pFile = fopen(fileName, "w");

        if (!pFile) {
            return false;
        }

        std::list<T>::iterator iter;

        for (iter = _results.begin(); iter != _results.end(); iter++) {
            T& value = *iter;        
            char * buff = NULL;
            int size = value.serialize(&buff);
            fwrite(buff, size, 1, pFile);
            free(buff);
        }

        fclose(pFile);
        return true;
    }


//class ReslultEntry {
//public:
//    virtual int serialize(char ** buff) { return 0;  }
//};
//
//class ResultsGrabber {
//public:
//    explicit ResultsGrabber();
//    virtual ~ResultsGrabber();
//
//    bool addResult(ReslultEntry& value);
//
//    bool renderResults(rnd::Renderer* renderer);
//
//    bool flushResultsToFile(const char* fileName);
//private:
//    std::list<ReslultEntry> _results;
//};
//
//ResultsGrabber::ResultsGrabber()
//{
//    ;
//}
//
//ResultsGrabber::~ResultsGrabber()
//{
//    _results.clear();
//}
//
//bool ResultsGrabber::addResult(ReslultEntry& value)
//{
//    _results.push_back(value);
//    return true;
//}
//
//bool ResultsGrabber::renderResults(rnd::Renderer* renderer)
//{
//    std::list<ReslultEntry>::iterator iter;
//
//    for (iter = _results.begin(); iter != _results.end(); iter++) {
//        ReslultEntry& value = *iter;        
//    }
//    
//    return true;
//}
//
//bool ResultsGrabber::flushResultsToFile(const char * fileName)
//{
//    FILE * pFile = fopen(fileName, "w");
//
//    if (!pFile) {
//        return false;
//    }
//
//    std::list<ReslultEntry>::iterator iter;
//
//    for (iter = _results.begin(); iter != _results.end(); iter++) {
//        ReslultEntry& value = *iter;        
//        char * buff = NULL;
//        int size = value.serialize(&buff);
//        fwrite(buff, size, 1, pFile);
//        delete [] buff;
//    }
//
//    fclose(pFile);
//    return true;
//}


} // namespace res

#endif //_RESULTS_GRABBER_H_
