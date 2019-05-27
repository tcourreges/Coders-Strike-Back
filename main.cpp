#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

constexpr int maxThrust = 100;
constexpr int boostAngleThreshold = 15;
constexpr float checkpointRadiusSqr = 360000.f;

// -------------------------------------------------------------------
// basic 2D vector to simplify notations
struct Vector2
{
    int x;
    int y;
    Vector2(int _x, int _y) : x(_x), y(_y) {}
};
bool operator==(const Vector2& a, const Vector2& b)
{
    return a.x == b.x && a.y == b.y;
}
Vector2 operator+(const Vector2& a, const Vector2& b)
{
    return Vector2(a.x+b.x, a.y+b.y);
}
Vector2 operator-(const Vector2& a, const Vector2& b)
{
    return Vector2(a.x-b.x, a.y-b.y);
}
Vector2 operator*(float k, const Vector2& a)
{
    return Vector2(k*a.x, k*a.y);
}
float distSqr(const Vector2& a, const Vector2& b)
{
    return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
}
// -------------------------------------------------------------------
// we don't know the checkpoints in advance (they are discovered from the input, one after the other)
// however, since they are repeated several times (laps), we can remember them (and know everything from lap 2 onwards)
// this is done by feeding the current checkpoint to the CheckpointManager, so that it can store/compute everything
class CheckpointManager
{
    public:
        int ShouldUseBoost() { return checkpointIndex == bestBoostIndex; }

        void Update(const Vector2& nextCheckpoint)
        {
            // init
            if (checkpoints.empty())
            {
                checkpoints.push_back(nextCheckpoint);
                return;
            }

            cerr << "Lap " << currentLap << " - Index " << checkpointIndex << " - Boost index " << bestBoostIndex << endl;

            if (nextCheckpoint == checkpoints[checkpointIndex]) // no update
                return;

            if (nextCheckpoint == checkpoints[0]) // end of the lap
            {
                if (currentLap == 0) // we now know all the checkpoints
                    ComputeBestBoostIndex();
                ++currentLap;
                checkpointIndex = 0;
            }
            else // next checkpoint
            {
                ++checkpointIndex;
                if (currentLap == 0)
                    checkpoints.push_back(nextCheckpoint);
            }
        }

        CheckpointManager() : currentLap(0), checkpointIndex(0), bestBoostIndex(-1) {}

    private:
        // simple function to compute the longest step (between 2 consecutive waypoints of the lap) so we know the best time to use our boost          
        void ComputeBestBoostIndex()
        {
            if (checkpoints.size() < 2)
                cerr << "forgot to collect the checkpoints";

            float longestDist = 0.f;

            for (size_t i=0; i < checkpoints.size(); i++)
            {
                const size_t j = (i+1) % checkpoints.size();
                float curDist = distSqr(checkpoints[i], checkpoints[j]);

                if (curDist > longestDist)
                {
                    bestBoostIndex = j;
                    longestDist = curDist;
                }
            }
        }

    private:
        vector<Vector2> checkpoints;
        int currentLap;
        int checkpointIndex;

        int bestBoostIndex;
};
// -------------------------------------------------------------------

int main()
{
    bool canBoost = true;
    CheckpointManager checkpoints;

    Vector2 prevPos{-1, -1};

    bool init = false;

    // game loop
    while (1) {
        int x;
        int y;
        int nextCheckpointX; // x position of the next check point
        int nextCheckpointY; // y position of the next check point
        int nextCheckpointDist; // distance to the next checkpoint
        int nextCheckpointAngle; // angle between your pod orientation and the direction of the next checkpoint
        cin >> x >> y >> nextCheckpointX >> nextCheckpointY >> nextCheckpointDist >> nextCheckpointAngle; cin.ignore();
        int opponentX;
        int opponentY;
        cin >> opponentX >> opponentY; cin.ignore();
        // ------------ end of the input ------------

        const Vector2 pos{x, y};
        const Vector2 cp{nextCheckpointX, nextCheckpointY};
        const int angle = abs(nextCheckpointAngle);

        if (prevPos.x < 0)
            prevPos = pos;

        checkpoints.Update(cp);

        // let's compute this turn:
        Vector2 target = cp;
        int thrust = maxThrust;
        bool useBoost = false;
        {
            if (angle<1) // we have the right trajectory, just go as fast as possible
            {
                thrust = maxThrust;

                // we can even try to use our boost
                useBoost = canBoost && checkpoints.ShouldUseBoost();
            }
            else // we need to adjust our trajectory
            {
                // try to avoid drifts
                const Vector2 dPos = pos - prevPos;
                target = cp - (3.f * dPos);

                // in order to align, we also need to reduce the thrust...

                // ...the closer we are to the checkpoint...
                const float distToCpSqr = distSqr(pos, cp);
                const float distanceSlowdownFactor = clamp(distToCpSqr/(checkpointRadiusSqr * 4.f), 0.f, 1.f);

                // ...and the less aligned we are
                const float angleSlowdownFactor = 1.f - clamp(angle/90.f, 0.f, 1.f);

                cerr << "Slowdown: distance " << distanceSlowdownFactor << " - angle " << angleSlowdownFactor << endl;

                thrust = maxThrust * distanceSlowdownFactor * angleSlowdownFactor;
            }
        }

        // update values for next turn
        {
            prevPos = pos;
            if (useBoost)
                canBoost = false;
        }

        // output the result
        string thrustString = useBoost ? "BOOST" : to_string(thrust);
        cerr << "thrust: " << thrustString;
        cout << target.x << " " << target.y << " " << thrustString << endl;
    }
}
