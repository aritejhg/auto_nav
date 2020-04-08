#include <cstdint>
#include <cmath>
#include <vector>
#include <stack>
#include <algorithm>

#define MAXSIZEX    512
#define MAXSIZEY    512

#define UNMAPPED    0
#define MAPPED      1
#define WALL        2
#define FRONTIER    3
#define ENDPOINT    4

#define LOOPTHROUGH_2D(i, imax, j, jmax) \
    for (int (i) = 1; (i) < (imax) - 1; (i)++) for (int (j) = 1; (j) < (jmax) - 1; (j)++)
#define ANY_NEARBY(arr, i, j, val) \
    ((arr)[(i)-1][(j)-1]==(val) || (arr)[(i)-1][(j)]==(val) || (arr)[(i)-1][(j)+1]==(val) || arr[(i)][(j)-1]==(val) ||\
    (arr)[(i)][(j)+1]==(val) || (arr)[(i)+1][(j)-1]==(val) || (arr)[(i)+1][(j)]==(val) || arr[(i)+1][(j)+1]==(val))
#define FIND_NEXT_NEARBY(arr, i, j, val) \
    (arr)[(i)][(j)+1]==(val) ? 0x0001 : (arr)[(i)+1][(j)+1]==(val) ? 0x0101 : (arr)[(i)+1][(j)]==(val) ? 0x0100 : \
    (arr)[(i)+1][(j)-1]==(val) ? 0x01FF : (arr)[(i)][(j)-1]==(val) ? 0x00FF : (arr)[(i)-1][(j)-1]==(val) ? 0xFFFF : \
    (arr)[(i)-1][(j)]==(val) ? 0xFF00 : (arr)[(i)-1][(j)+1]==(val) ? 0xFF01 : 0x0000

using namespace std;
typedef struct {
    int x;
    int y;
} point_t;

/* FUNCTION DECLARATIONS AND INTERFACES */

void implMapDilateErode (uint8_t *mapArr, int height, int width);
extern "C" void MapDilateErode (uint8_t *mapArr, int height, int width) 
    {implMapDilateErode(mapArr, height, width);}

int implFindNavGoal (uint8_t *mapArr, int height, int width, int posY, int posX, int *res, float *resDir);
extern "C" int FindNavGoal (uint8_t *mapArr, int height, int width, int posY, int posX, int *res, float *resDir)
    {return implFindNavGoal (mapArr, height, width, posY, posX, res, resDir);}

point_t findCentre (vector<point_t> pts);
float findDirection (point_t a, point_t b);
float diffSmallerDirection (float a, float b);

/* FUNCTION IMPLEMENTATIONS */

void implMapDilateErode (uint8_t *mapArr, int height, int width)
{
    uint8_t tmpMapArr[MAXSIZEY][MAXSIZEX]; 
    LOOPTHROUGH_2D(i, height, j, width) tmpMapArr[i][j] = mapArr[i*width+j];

    LOOPTHROUGH_2D(i, height, j, width)  {
        if (tmpMapArr[i][j] != WALL && ANY_NEARBY(tmpMapArr, i, j, WALL))
            mapArr[i*width+j] = WALL;  // Dilate the walls
        else if (tmpMapArr[i][j] == UNMAPPED && ANY_NEARBY(tmpMapArr, i, j, MAPPED))
            mapArr[i*width+j] = MAPPED;  // Erode the unmapped
    }
}

int implFindNavGoal (uint8_t *mapArr, int height, int width, int posY, int posX, int *res, float *resDir)
{
    uint8_t mapA[MAXSIZEY][MAXSIZEX], mapB[MAXSIZEY][MAXSIZEX];
    LOOPTHROUGH_2D(i, height, j, width) mapA[i][j] = mapArr[i*width+j];
    
    // 1st step: find unmapped frontiers and their endpoint pixels
    LOOPTHROUGH_2D(i, height, j, width) {
        if (mapA[i][j] == UNMAPPED && ANY_NEARBY(mapA, i, j, MAPPED)) mapB[i][j] = FRONTIER;
        else mapB[i][j] = mapA[i][j];
    }
    LOOPTHROUGH_2D(i, height, j, width) {
        if (mapB[i][j] == UNMAPPED && ANY_NEARBY(mapB, i, j, FRONTIER)) mapA[i][j] = FRONTIER;
        else mapA[i][j] = mapB[i][j];
    }
    LOOPTHROUGH_2D(i, height, j, width) {
        if (mapA[i][j] == FRONTIER && ANY_NEARBY(mapA, i, j, WALL)) mapB[i][j] = ENDPOINT;
        else mapB[i][j] = mapA[i][j];
    }
    
    // 2nd step: find the endpoint coordinates
    vector<point_t> endpoints;
    LOOPTHROUGH_2D(i, height, j, width) {
        if (mapB[i][j] == ENDPOINT){
            point_t px = {.x = j, .y = i};
            vector<point_t> eppx;
            stack<point_t> trace;
            while (true) {
                eppx.push_back(px);
                trace.push(px);
                mapB[px.y][px.x] = FRONTIER;
                uint16_t movement = FIND_NEXT_NEARBY(mapB, px.y, px.x, ENDPOINT);
                while (movement == 0x0 && trace.size() > 1) {
                    trace.pop();
                    px = trace.top();
                    movement = FIND_NEXT_NEARBY(mapB, px.y, px.x, ENDPOINT);
                }
                if (movement == 0x0) break;
                else {
                    px.y += (int8_t)(movement >> 8);
                    px.x += (int8_t)(movement);
                }
            }
            endpoints.push_back(findCentre(eppx));
        }
    }
    if (endpoints.size() == 0)
        return 1;   // No Endpoint => Map COMPLETED
    for (auto p : endpoints)
        mapB[p.y][p.x] = ENDPOINT;
    
    // 3rd step: group the endpoints connected by frontiers
    vector< vector<point_t> > epGrps;
    LOOPTHROUGH_2D(i, height, j, width) {
        if (mapA[i][j] == FRONTIER) {
            point_t pt = {.x = j, .y = i};
            vector<point_t> eps;
            stack<point_t> trace;
            while (true) {
                trace.push(pt);
                if (mapB[pt.y][pt.x] == ENDPOINT)
                    eps.push_back(pt);
                mapA[pt.y][pt.x] = UNMAPPED;
                uint16_t mv = FIND_NEXT_NEARBY(mapA, pt.y, pt.x, FRONTIER);
                while (mv == 0x0 && trace.size() > 1) {
                    trace.pop();
                    pt = trace.top();
                    mv = FIND_NEXT_NEARBY(mapA, pt.y, pt.x, FRONTIER);
                }
                if (mv == 0x0) break;
                else {
                    pt.y += (int8_t)(mv >> 8);
                    pt.x += (int8_t)(mv);
                }
            }
            if (eps.size() >= 2) epGrps.push_back(eps);
        }
    }
    
    // 4th step: Find their mid-points and directions
    point_t posBot = {.x = posX, .y = posY};
    vector<point_t> midpoints;
    vector<float> dirs;
    for (auto& epg : epGrps) {
        if (epg.size() > 2) {
            // Usually there will be 2 endpoints for each frontier. For now, if there are
            // more endpoints, we use the two farthest from the robot.
            std::sort(epg.begin(), epg.end(), [posX, posY](point_t a, point_t b) {
                float da = sqrt(powf(a.x - posX, 2) + powf(a.y - posY, 2));
                float db = sqrt(powf(b.x - posX, 2) + powf(b.y - posY, 2));
                return da > db;
            });
            while (epg.size() > 2) epg.pop_back();
        }
        // Calculate midpoint
        point_t mp = {
            .x = (epg[0].x + epg[1].x) / 2,
            .y = (epg[0].y + epg[1].y) / 2
        };
        midpoints.push_back(mp);
        // Calculate direction
        float lineDir = findDirection(epg[0], epg[1]);
        float routeDir = findDirection(posBot, mp);
        float goalDir = 
            diffSmallerDirection(lineDir + M_PI_2, routeDir) < 
            diffSmallerDirection(lineDir - M_PI_2, routeDir) ?
            (lineDir + M_PI_2) : (lineDir - M_PI_2);
        goalDir = (goalDir > M_PI) ? goalDir - 2 * M_PI : 
            (goalDir < -M_PI) ? goalDir + 2 * M_PI : goalDir;
        dirs.push_back(goalDir);
    }
    std::sort(midpoints.begin(), midpoints.end(), [posX, posY](point_t a, point_t b) {
        float da = sqrt(powf(a.x - posX, 2) + powf(a.y - posY, 2));
        float db = sqrt(powf(b.x - posX, 2) + powf(b.y - posY, 2));
        return da < db;
    });

    // Output
    int count = 0;
    for (auto p : midpoints) {
        if (count >= res[0]) break;
        res[count * 2 + 1] = p.x;
        res[count * 2 + 2] = p.y;
        resDir[count] = dirs[count];
        count++;
    }
    res[0] = count;
    return 0;
}

point_t findCentre (vector<point_t> pts)
{
    if (pts.size() == 0)
        return (point_t){.x = 0, .y = 0};
    float xsum = 0, ysum = 0;
    for (auto p : pts) {
        xsum += p.x;
        ysum += p.y;
    }
    xsum /= pts.size();
    ysum /= pts.size();

    std::sort(pts.begin(), pts.end(), [xsum, ysum](point_t a, point_t b) {
        float da = sqrt(powf(a.x - xsum, 2) + powf(a.y - ysum, 2));
        float db = sqrt(powf(b.x - xsum, 2) + powf(b.y - ysum, 2));
        return da < db;
    });
    return pts.front();

    // point_t ctr = pts.back();
    // float mindist = sqrt(powf(pts.back().x - xsum, 2) + powf(pts.back().y - ysum, 2));
    // pts.pop_back();
    // while (!pts.empty()) {
    //     float currdist = sqrt(powf(pts.back().x - xsum, 2) + powf(pts.back().y - ysum, 2));
    //     ctr = (currdist < mindist) ? pts.back() : ctr;
    //     mindist = (currdist < mindist) ? currdist : mindist;
    //     pts.pop_back();
    // }
    // return ctr;
}

float findDirection (point_t a, point_t b)  // in radian
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float mag = sqrt(dx * dx + dy * dy);
    dy /= mag;  // cos(theta) == (0, 1) . (dx, dy)_unitvec = dy
    if (signbit(dx)) return -acos(dy);
    else return acos(dy);
}

float diffSmallerDirection (float a, float b)   // in radian
{
    float res = a - b;
    while (res < 0) res += 2 * M_PI;
    while (res > 2 * M_PI) res -= 2 * M_PI;
    if (res > M_PI) res = 2 * M_PI - res;
    return res;
}