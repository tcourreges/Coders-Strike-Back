#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>
#include <cstdlib>

using namespace std;

constexpr int maxThrust = 100;
constexpr int maxRotation = 18;

constexpr float checkpointRadius = 600.f;
constexpr float checkpointRadiusSqr = checkpointRadius * checkpointRadius;

constexpr float podRadius = 400.f;
constexpr float podRadiusSqr = podRadius * podRadius;
constexpr float PI = 3.14159f;

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
Vector2 operator+=(Vector2& a, const Vector2& b)
{
    a = a+b;
}
Vector2 operator-(const Vector2& a, const Vector2& b)
{
    return Vector2(a.x-b.x, a.y-b.y);
}
Vector2 operator*(float k, const Vector2& a)
{
    return Vector2(k*a.x, k*a.y);
}
Vector2 operator*=(Vector2& a, float k)
{
    a = k*a;
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

struct Pod
{
    int id = -1;

    // state
    Vector2 position;
    Vector2 speed;
    int angle = -1;

    // race status
    int checkpointId = 0;
    int checkpointsPassed = 0;

    // boost/shield
    bool canBoost = true;
    int shieldCd = 0;
};

void ManageShield(bool turnOn, Pod& p)
{
  if (turnOn)
    p.shieldCd = 4; // this turn + 3 more
  else if (p.shieldCd > 0)
    --p.shieldCd;
}

int Mass(const Pod& p)
{
    if (p.shieldCd == 4)
        return 10;
    return 1;
}

float CollisionTime(Pod& p1, Pod& p2)
{
    const Vector2 dP = (p2.position - p1.position);
    const Vector2 dS = (p2.speed - p1.speed);

    constexpr float eps = 0.000001f; // float precision...
    if (dot(dS,dS) < eps)
        return INFINITY;

    // we're looking for t such that:
    // |            p2(t)           -           p1(t)           | < |2*podRadiusSqr|
    // |(p2.position + t*p2.speed)  - (p1.position + t*p1.speed)| < |2*podRadiusSqr|
    // |(p2.position - p1.position) -  t*(p2.speed - p1.speed)  | < |2*podRadiusSqr|
    // |         dP                 +            t*dS           | < |2*podRadiusSqr|
    // t^2 dS^2 + t 2dPdS + dP^2 < 4 podRadiusSqr^2
    // t^2  a   + t   b   +   c      = 0;
    const float a = dot(dS, dS);
    const float b = -2.f*dot(dP, dS);
    const float c = dot(dP,dP) - 4.f*podRadiusSqr;

    if (a < eps)
        return INFINITY;

    const float delta = b*b - 4.f*a*c;
    if (delta < 0.f)
        return INFINITY;

    const float t = (b - sqrt(delta)) / (2.f * a);
    if (t <= eps)
        return INFINITY;

    return t;
}

void Rebound(Pod& a, Pod& b)
{
    // https://en.wikipedia.org/wiki/Elastic_collision#Two-dimensional_collision_with_two_moving_objects
    const float mA = Mass(a);
    const float mB = Mass(b);

    const Vector2 dP = b.position - a.position;
    const float AB = dist(a.position, b.position);

    const Vector2 u = (1.f / AB) * dP; // rebound direction

    const Vector2 dS = b.speed - a.speed;

    const float m = (mA * mB) / (mA + mB);
    const float k = dot(dS, u);

    const float impulse = -2.f * m * k;
    cerr << "impulse: " << impulse << endl;
    constexpr float minImpulse = 120.f;
    const float impulseToUse = clamp(impulse, -minImpulse, minImpulse);

    a.speed += (-1.f/mA) * impulse * u;
    b.speed += (1.f/mB) * impulse * u;
}

void print(const Pod& p)
{
    //if(p.id != 1) return;

    cerr << "position: " << p.position << endl;
    cerr << "speed:    " << p.speed << endl;
    cerr << "angle:    " << p.angle << endl;
    cerr << "nextCpId: " << p.checkpointId;
    cerr << "  passed: " << p.checkpointsPassed << endl;
    cerr << "canBoost: " << p.canBoost;
    cerr << "  shieldCd: " << p.shieldCd << endl << endl;
}

void UpdatePodFromInput(Pod& p, int id)
{
    int x, y, vx, vy, angle, nextCheckPointId;
    cin >> x >> y >> vx >> vy >> angle >> nextCheckPointId; cin.ignore();

    p.id = id;
    p.position = Vector2(x,y);
    p.speed = Vector2(vx, vy);
    p.angle = angle;
    if (p.checkpointId != nextCheckPointId)
        ++p.checkpointsPassed;
    p.checkpointId = nextCheckPointId;
}

// move of 1 pod, 1 turn
struct Move
{
    int rotation = 0; // -maxRotation, maxRotation
    int thrust = 0;; // 0, 100
    bool shield = false;
    bool boost = false;
};

// all the moves of 1 turn
struct Turn
{
    Move moves[2];
};

#define SIMULATIONTURNS 1
// all the moves to perform the next SIMULATIONTURNS turns
struct Solution
{
    Turn turns[SIMULATIONTURNS];
};

class Simulation
{
    private:
        vector<Vector2> checkpoints;
        int checkpointCount;
        int maxCheckpoints;

    public:
        Vector2 InitCheckpointsFromInput()
        {
            int laps;
            {
                cin >> laps; cin.ignore();
                cin >> checkpointCount; cin.ignore();
            }

            checkpoints.reserve(checkpointCount);
            for (int i = 0; i < checkpointCount; i++)
            {
                int checkpointX, checkpointY;
                cin >> checkpointX >> checkpointY; cin.ignore();
                checkpoints[i] = Vector2(checkpointX, checkpointY);
            }

            maxCheckpoints = laps*checkpointCount;

            return checkpoints[1];
        }

        int TestSolution(vector<Pod>& pods, Solution s, int step)
        {
            for (int i=0; i<SIMULATIONTURNS; i++)
            {
                cerr << "====================" << endl;
                cerr << "turn: " << step << " + " << i << endl;
                cerr << "====================" << endl;
                PlayOneTurn(pods, s.turns[i]);
            }

            return RateSolution(pods);
        }

    private:
        int RateSolution(vector<Pod>& pods)
        {
            // TODO
            return 0;
        }

        void PlayOneTurn(vector<Pod>& pods, const Turn& moves)
        {
            Rotate(pods, moves);
            Accelerate(pods, moves);
            Move(pods);
            Friction(pods);
            EndTurn(pods);
        }

        void Rotate(vector<Pod>& pods, const Turn& turn)
        {
            for (int i=0; i<2; i++)
            {
                Pod& p = pods[i];
                //print(p);

                const int rotation = turn.moves[i].rotation;
                //cerr << "pod " << p.id << " rotates by " << rotation << endl;
                p.angle = (p.angle + rotation) % 360;
            }
        }
        void Accelerate(vector<Pod>& pods, const Turn& turn)
        {
            for (int i=0; i<2; i++)
            {
                Pod& p = pods[i];

                // shield
                ManageShield(turn.moves[i].shield, p);
                if (p.shieldCd > 0)
                    continue; // no thrust this turn

                const float angleRad = p.angle * PI / 180.f;
                const Vector2 direction{cos(angleRad), sin(angleRad)};

                //cerr << direction << endl;

                int thrust = turn.moves[i].thrust;

                //boost
                if (p.canBoost && turn.moves[i].boost)
                {
                    thrust = 650;
                    p.canBoost = false;
                }

                p.speed += thrust * direction;
            }
        }
        void Move(vector<Pod>& pods)
        {
            float t = 0.f;
            constexpr float turnEnd = 1.f;
            while (t < turnEnd)
            {
                Pod* a = nullptr;
                Pod* b = nullptr; // first pods to collide with each other
                float dt = turnEnd - t;
                for (int i=0; i<4; i++)
                {
                    for (int j=i+1; j<4; j++)
                    {
                        const float collisionTime = CollisionTime(pods[i], pods[j]);
                        if ( (t+collisionTime < turnEnd) && (collisionTime < dt) )
                        {
                            dt = collisionTime;
                            a = &pods[i];
                            b = &pods[j];
                        }
                    }
                }

                // move all the pods until the date we selected
                for (Pod& p : pods)
                {
                    p.position += dt * p.speed;

                    // checkpoints (do we need to be more accurate?)
                    if (distSqr(p.position, checkpoints[p.checkpointId]) < checkpointRadiusSqr)
                    {
                        p.checkpointId = (p.checkpointId + 1) % checkpointCount;
                        ++p.checkpointsPassed;
                    }
                }

                // a and b are colliding -> elastic collision
                if (a != nullptr && b != nullptr)
                {
                    cerr << "collision at t=" << t << " + dt=" << dt;
                    cerr << " between "<<a->id<<" and "<<b->id<<endl;
                    Rebound(*a, *b);
                }

                t+=dt;
            }
        }
        void Friction(vector<Pod>& pods)
        {
            for (Pod& p : pods)
            {
                p.speed *= 0.85f;
            }
        }
        void EndTurn(vector<Pod>& pods)
        {
            for (Pod& p : pods)
            {
                p.speed = Vector2{ (int) p.speed.x, (int) p.speed.y };
                p.position = Vector2{round(p.position.x), round(p.position.y)};

                //print(p);
            }
        }
};

void ConvertSolutionToOutput(const Solution& sol, vector<Pod>& pods)
{
    // get the first turn
    const Turn t = sol.turns[0];

    for (int i=0; i<2; i++)
    {
        const Pod& p = pods[i];

        // generate coordinates from the rotation
        const float angle = (p.angle + t.moves[i].rotation) % 360;
        const float angleRad = angle * PI / 180.f;

        const Vector2 direction{10000*cos(angleRad),10000*sin(angleRad)};
        const Vector2 target = p.position + direction;
        cout << round(target.x) << " " << round(target.y) << " ";

        if (t.moves[i].shield)
            cout << "SHIELD";
        else if (t.moves[i].boost)
            cout << "BOOST";
        else
            cout << t.moves[i].thrust;
        cout << endl;
    }

    // those values won't be updated by the input, we have to do it
    // TODO: move that elsewhere
    for (int i=0; i<2; i++)
    {
        Pod& p = pods[i];
        ManageShield(t.moves[i].shield, p);

        if (p.shieldCd == 0 && t.moves[i].boost)
            p.canBoost = false;
    }
}

inline int rnd(int a, int b)
{
    return (std::rand() % (b-a)) + a;
}

int main()
{
    Simulation simulation;
    Vector2 firstCheckpoint = simulation.InitCheckpointsFromInput();

    vector<Pod> pods(4);

    int step=0;
    while (1)
    {
        for (int i=0; i<4; i++)
        {
            UpdatePodFromInput(pods[i], i);
            if (step == 0) // first turn, override the angle
            {
                const Vector2 dir = (firstCheckpoint - pods[i].position).normalized();
                float a = acos(dir.x) * 180.f / PI;
                if(dir.y<0) a = (360.f - a);
                pods[i].angle = a;
            }
            //print(pods[i]);
        }

        // let's try a random solution
        Solution s;
        {
            for (int i=0; i<2; i++)
            {
                Move& m = s.turns[0].moves[i];
                m.rotation = rnd(-maxRotation, maxRotation);
                m.thrust = rnd(0, maxThrust);
                m.shield = rnd(0, 100) > 90;
                m.boost = rnd(0,100) > 90;
            }

            vector<Pod> podsCopy = pods;
            simulation.TestSolution(podsCopy, s, step);
        }

        ConvertSolutionToOutput(s, pods);

        ++step;
    }
}
