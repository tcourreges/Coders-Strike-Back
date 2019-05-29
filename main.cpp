#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>

using namespace std;

constexpr int maxThrust = 100;
constexpr float checkpointRadius = 600.f;
constexpr float PI = 3.14;

// -------------------------------------------------------------------
// basic 2D vector to simplify notations
struct Vector2
{
    float x;
    float y;
    Vector2(float _x, float _y) : x(_x), y(_y) {}
    Vector2() : x(0.f), y(0.f) {}

    Vector2 normalized() const
    {
        const float norm = sqrt(x*x+y*y);
        return Vector2(x/norm, y/norm);
    }
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
ostream& operator<<(ostream& os, const Vector2& v)
{
    os << v.x << " " << v.y;
    return os;
}
float distSqr(const Vector2& a, const Vector2& b)
{
    return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
}
float dist(const Vector2& a, const Vector2& b)
{
    return sqrt(distSqr(a, b));
}
float dot(const Vector2& a, const Vector2& b)
{
    return a.x*b.x + a.y*b.y;
}
// -------------------------------------------------------------------

class Pod
{
    private:
        Vector2 pos;
        Vector2 dPos;
        int angle;
        int checkpointId;

        bool canBoost;
        int shieldCooldown;

        Vector2 target;
        int thrust;

        bool useBoost;
        bool useShield;

    public:
        Pod() :
            pos(0, 0)
            , dPos(0, 0)
            , angle(0)
            , checkpointId(0)
            , canBoost(true)
            , shieldCooldown(0)
            , target(0, 0)
            , thrust(maxThrust)
            , useBoost(false)
            , useShield(false)
        {
        }

        void output() const
        {
            cout << target.x << " " << target.y << " ";
            if (useBoost)
                cout << "BOOST";
            else if (useShield)
                cout << "SHIELD";
            else if (shieldCooldown > 0)
                cout << 0;
            else
                cout << thrust;
            cout << endl;
        }
        //------------------------------------------------------------
        const Vector2& Position() const { return pos; }
        const Vector2& Speed() const { return dPos; }
        float Angle() const { return angle; }
        float CheckpointId() const { return checkpointId; }
        const Vector2& Target() const { return target; }
        //------------------------------------------------------------
        void SetThrust(int parThrust) { thrust = parThrust; }
        void SetTarget(const Vector2& parTarget) { target = parTarget; }
        void RequestShield()
        {
           useShield = true;
           shieldCooldown = 3;
        }
        void RequestBoost()
        {
            if (shieldCooldown > 0)
                return;
            if (!canBoost)
                return;
            useBoost = true;
            canBoost = false;
        }
        //------------------------------------------------------------

        // i moved the cin management given to this function for readability
        void UpdatePodFromInput()
        {
            int x; // x position of your pod
            int y; // y position of your pod
            int vx; // x speed of your pod
            int vy; // y speed of your pod
            int angle; // angle of your pod
            int nextCheckPointId; // next check point id of your pod
            cin >> x >> y >> vx >> vy >> angle >> nextCheckPointId; cin.ignore();
            // -------------------------------------
            this->pos = Vector2(x,y);
            this->dPos = Vector2(vx, vy);
            this->angle = angle;
            this->checkpointId = nextCheckPointId;

            // also update those values
            this->useShield = false;
            this->useBoost = false;
            if (this->shieldCooldown > 0)
                --this->shieldCooldown;
        }
};

int ComputeBestBoostIndex(const vector<Vector2>& checkpoints)
{
    if (checkpoints.size() < 2)
        cerr << "forgot to collect the checkpoints";

    int bestBoostIndex = 0;
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

    return bestBoostIndex;
}

bool ShouldUseShield(const Pod& a, const Pod& b)
{
    const Vector2 aNextPos = a.Position()+a.Speed();
    const Vector2 bNextPos = b.Position()+b.Speed();
    const bool collision = distSqr(aNextPos, bNextPos) < (2*420)*(2*420);
    if (!collision)
        return false;

    const Vector2 dirTarget = (a.Target() - a.Position()).normalized();
    const Vector2 dirB = (b.Position() - a.Position()).normalized();
    return dot(dirTarget, dirB) > 0.3f;
}

bool ComputeThrust(Pod& p, int checkpointToBoost)
{
    if (p.Angle()<1) // we have the right trajectory, just go as fast as possible
    {
        p.SetThrust(maxThrust);

        // we can even try to use our boost
        if (p.CheckpointId() == checkpointToBoost)
            p.RequestBoost();
    }
    else // we need to adjust our trajectory
    {
        const Vector2 cp = p.Target();

        // try to avoid drifts
        p.SetTarget(cp - (3.f * p.Speed()));

        // in order to align, we also need to reduce the thrust

        const float distToCp = dist(cp, p.Position());
        const float distanceSlowdownFactor = clamp(distToCp/(2.f * checkpointRadius), 0.f, 1.f);

        const float cpAngle = [&]() -> float
        {
            const Vector2 dirToCp = (cp - p.Position()).normalized();
            float res = acos(dirToCp.x) * 180.f / PI;
            if (dirToCp.y < 0.f)
                return 360.f - res;
            else
                return res;
        }();
        const float angle = cpAngle - p.Angle();
        const float angleSlowdownFactor = 1.f - clamp(abs(angle)/90.f, 0.f, 1.f);

        cerr << "Slowdown: distance " << distanceSlowdownFactor << " - angle " << angleSlowdownFactor << endl;
        p.SetThrust(maxThrust * distanceSlowdownFactor * angleSlowdownFactor);
    }
}

int main()
{
    vector<Pod> myPods(2);
    vector<Pod> opponentPods(2);

    // read checkpoint information
    int laps;
    int checkpointCount;
    {
        cin >> laps; cin.ignore();
        cin >> checkpointCount; cin.ignore();
    }
    vector<Vector2> checkpoints(checkpointCount);
    for (int i = 0; i < checkpointCount; i++)
    {
        int checkpointX;
        int checkpointY;
        cin >> checkpointX >> checkpointY; cin.ignore();
        checkpoints[i] = Vector2(checkpointX, checkpointY);
    }
    const int checkpointToBoost = ComputeBestBoostIndex(checkpoints);

    // game loop
    while (1)
    {
        for (int i = 0; i < 2; i++)
            myPods[i].UpdatePodFromInput();
        for (int i = 0; i < 2; i++)
            opponentPods[i].UpdatePodFromInput();

        for (int i=0; i<2; i++)
        {
            cerr << endl << "=== Pod " << i << " ==========" << endl;
            Pod& p = myPods[i];

            p.SetTarget(checkpoints[p.CheckpointId()]);

            if (ShouldUseShield(p, myPods[1-i])
                || ShouldUseShield(p, opponentPods[0])
                || ShouldUseShield(p, opponentPods[1]))
                p.RequestShield();

            ComputeThrust(p, checkpointToBoost);
        }

        myPods[0].output();
        myPods[1].output();
    }
}
