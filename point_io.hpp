#ifndef POINTIO_H
#define POINTIO_H

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <limits>

#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/BufferReader.hpp>

struct XYZ {
    float x;
    float y;
    float z;
};

struct Extent{
    double minx;
    double maxx;
    double miny;
    double maxy;

    Extent(){
        minx = miny = (std::numeric_limits<double>::max)();
        maxx = maxy = (std::numeric_limits<double>::lowest)();
    }

    void inline update(double x, double y){
        minx = (std::min)(minx, x);
        maxx = (std::max)(maxx, x);
        miny = (std::min)(miny, y);
        maxy = (std::max)(maxy, y);
    }

    double width() const {
        return maxx - minx;
    }

    double height() const {
        return maxy - miny;
    }

    friend std::ostream& operator<<(std::ostream &out, const Extent &e){
        return out << std::setprecision(12) << "[minx: " << e.minx << 
                                        ", maxx: " << e.maxx <<
                                        ", miny: " << e.miny <<
                                        ", maxy: " << e.maxy << "]";

    }
};


struct PointSet {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;

    pdal::PointViewPtr pointView = nullptr;

    inline size_t count() const { return x.size(); }
    inline size_t size() const { return x.size(); }
    inline void resize(size_t count){ 
        x.resize(count);
        y.resize(count);
        z.resize(count);
    }
    

    ~PointSet() {
    }

    Extent extent;
    pdal::SpatialReference srs;
};

std::string getVertexLine(std::ifstream &reader);
size_t getVertexCount(const std::string &line);
inline void checkHeader(std::ifstream &reader, const std::string &prop);
inline bool hasHeader(const std::string &line, const std::string &prop);

PointSet *fastPlyReadPointSet(const std::string &filename, size_t decimation = 1);
PointSet *pdalReadPointSet(const std::string &filename, uint8_t onlyClass = 255, size_t decimation = 1);
PointSet *readPointSet(const std::string &filename, int classification = -1, int decimation = 1);

bool fileExists(const std::string &path);


#endif
