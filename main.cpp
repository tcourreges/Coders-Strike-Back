#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>
#include <cstdlib>
#include <chrono>

using namespace std;
using namespace std::chrono;

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

    int score = 0;
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
    //cerr << "impulse: " << impulse << endl;
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

    //print(p);
}

constexpr int SIMULATIONTURNS = 4;
struct Move
{
    int rotation = 0; // -maxRotation, maxRotation
    int thrust = 0; // 0, 100
    bool shield = false;
    bool boost = false;
};
struct Turn
{
    vector<Move> moves = vector<Move>(2);

    Move& operator[](size_t idx)             { return moves[idx]; }
    const Move& operator[](size_t idx) const { return moves[idx]; }
};
// all the moves to perform the next SIMULATIONTURNS turns
struct Solution
{
    vector<Turn> turns = vector<Turn>(SIMULATIONTURNS);

    Turn& operator[](size_t idx)             { return turns[idx]; }
    const Turn& operator[](size_t idx) const { return turns[idx]; }

    int score = -1;
};

void print(const Solution& s)
{
    for(int t=0; t<SIMULATIONTURNS; t++)
    {
        for(int i=0; i<2; i++)
        {
            const Move& m = s[t][i];
            cerr << m.rotation << "     " << m.thrust << endl;
        }
        cerr << endl;
    }
}

class Simulation
{
    private:
        vector<Vector2> checkpoints;
        int checkpointCount;
        int maxCheckpoints;

    public:
        int MaxCheckpoints() { return maxCheckpoints; }
        const vector<Vector2>& Checkpoints() { return checkpoints; }

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

        int PlaySolution(vector<Pod>& pods, Solution s)
        {
            for (int i=0; i<SIMULATIONTURNS; i++)
            {
                PlayOneTurn(pods, s[i]);
            }
        }

    private:
        void PlayOneTurn(vector<Pod>& pods, const Turn& moves)
        {
            Rotate(pods, moves);
            Accelerate(pods, moves);
            UpdatePositions(pods);
            Friction(pods);
            EndTurn(pods);
        }

        void Rotate(vector<Pod>& pods, const Turn& turn)
        {
            for (int i=0; i<2; i++)
            {
                Pod& p = pods[i];
                const Move& m = turn[i];
                //print(p);

                const int rotation = m.rotation;
                //cerr << "pod " << p.id << " rotates by " << rotation << endl;
                p.angle = (p.angle + rotation) % 360;
            }
        }
        void Accelerate(vector<Pod>& pods, const Turn& turn)
        {
            for (int i=0; i<2; i++)
            {
                Pod& p = pods[i];
                const Move& m = turn[i];

                // shield
                ManageShield(m.shield, p);
                if (p.shieldCd > 0)
                    continue; // no thrust this turn

                const float angleRad = p.angle * PI / 180.f;
                const Vector2 direction{cos(angleRad), sin(angleRad)};

                //cerr << direction << endl;

                int thrust = m.thrust;

                //boost
                if (p.canBoost && m.boost)
                {
                    thrust = 650;
                    p.canBoost = false;
                }

                p.speed += thrust * direction;
            }
        }
        void UpdatePositions(vector<Pod>& pods)
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
                    //cerr << "collision at t=" << t << " + dt=" << dt;
                    //cerr << " between "<<a->id<<" and "<<b->id<<endl;
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
    const int t = 0; // first turn

    for (int i=0; i<2; i++)
    {
        const Pod& p = pods[i];
        const Move& m = sol[t][i];

        // generate coordinates from the rotation
        cerr << "rotation: " << m.rotation;;

        const float angle = (p.angle + m.rotation) % 360;
        const float angleRad = angle * PI / 180.f;

        const Vector2 direction{10000*cos(angleRad),10000*sin(angleRad)};
        const Vector2 target = p.position + direction;
        cout << round(target.x) << " " << round(target.y) << " ";

        if (m.shield)
            cout << "SHIELD";
        else if (m.boost)
            cout << "BOOST";
        else
            cout << m.thrust;
            cerr << " thrust: " << m.thrust << endl;

        cout << endl;
    }

    // those values won't be updated by the input, we have to do it
    // TODO: move that elsewhere
    for (int i=0; i<2; i++)
    {
        Pod& p = pods[i];
        const Move& m = sol[0][i];

        ManageShield(m.shield, p);

        if (p.shieldCd == 0 && m.boost)
            p.canBoost = false;
    }
}

// https://software.intel.com/en-us/articles/fast-random-number-generator-on-the-intel-pentiumr-4-processor
// apparently much faster than std::rand
inline int fastrand()
{
    static unsigned int g_seed = 100;
    g_seed = (214013*g_seed+2531011);
    return (g_seed>>16)&0x7FFF;
}
inline int rnd(int a, int b)
{
    return (fastrand() % (b-a)) + a;
}

constexpr int SOLUTIONCOUNT = 6;

class Solver
{
    private:
        vector<Solution> solutions;
        Simulation* sim;


    public:
        Solver(Simulation* parSimulation)
        {
            sim = parSimulation;

            solutions.resize(2*SOLUTIONCOUNT);
            for (int s=0; s<SOLUTIONCOUNT; s++)
            {
                for (int t=0; t<SIMULATIONTURNS; t++)
                {
                    for (int i=0; i<2; i++)
                    {
                        Move& m = solutions[s][t][i];
                        Randomize(m);
                    }
                }
            }

            // hack: just set the boost here if possible, don't waste time later since there's only 1 boost per race anyway...
            const float d = dist(sim->Checkpoints()[0], sim->Checkpoints()[1]);
            if (d < 3000)
                return;
            cerr << "Boosting the first turn" << endl;
            for (int i=0; i<2; i++)
            {
                for (int s=0; s<SOLUTIONCOUNT; ++s)
                    solutions[s][0][i].boost = true;
            }
        }

        Solution Solve(const vector<Pod>& pods, int time)
        {
            auto start = high_resolution_clock::now();

            // init
            for (int i=0; i<SOLUTIONCOUNT; ++i)
            {
                Solution& s = solutions[i];
                ShiftByOneTurn(s);
                ComputeScore(s, pods);
            }

            int step = 0;
            while(1)
            {
                for (int i=0; i<SOLUTIONCOUNT; ++i)
                {
                    Solution& newSol = solutions[SOLUTIONCOUNT+i];
                    newSol = solutions[i];
                    Mutate(newSol);
                    ComputeScore(newSol, pods);
                }

                // sort the solutions by score
                std::sort( solutions.begin(), solutions.end(),
                          [](const Solution& a, const Solution& b) { return a.score > b.score; }
                         );
                ++step;

                // time's up
                auto now = high_resolution_clock::now();
                auto d = std::chrono::duration_cast<milliseconds>(now - start);
                if (d.count() >= time)
                    break;
            }

            cerr << step << " evolution steps - best score: " << solutions[0].score << endl;
            return solutions[0];
        }

    private:
        void Randomize(Move& m, bool modifyAll = true)
        {
            // if !all, we need to select which value to modify

            // TODO: find a cleaner way to rewrite this (but keep the probabilities compile time)...
            constexpr int all=-1, rotation=0, thrust=1, shield=2, boost=3;
            constexpr int         pr=5      , pt=pr+5,  ps=pt+1,  pb=ps+0;

            const int i = modifyAll ? -1 : rnd(0, pb);
            const int valueToModify = [i]() -> int
            {
                if (i == -1)
                    return true;
                if (i<=pr)
                    return rotation;
                if (i<=pt)
                    return thrust;
                if (i<=ps)
                    return shield;
                return boost;
            }();
            auto modifyValue = [valueToModify](int parValue)
            {
                if(valueToModify == all)
                    return true;
                return valueToModify == parValue;
            };

            if (modifyValue(rotation))
            {
                const int r = rnd(-2*maxRotation, 3*maxRotation);
                if (r > 2*maxRotation)
                    m.rotation = 0;
                else
                    m.rotation = clamp(r, -maxRotation, maxRotation);
            }
            if (modifyValue(thrust))
            {
                const int r = rnd(-0.5f * maxThrust, 2*maxThrust);
                m.thrust = clamp(r, 0, maxThrust);
            }
            if (modifyValue(shield))
            {
                m.shield = !m.shield;
            }
            if (modifyValue(boost))
            {
                m.boost = !m.boost;
            }
        }

        void ShiftByOneTurn(Solution& s)
        {
            for (int t=1; t<SIMULATIONTURNS; t++)
                for (int i=0; i<2; i++)
                    s[t-1][i] = s[t][i];

            // create a new random turn at the end
            for (int i=0; i<2; i++)
            {
                Move& m = s[SIMULATIONTURNS-1][i];
                Randomize(m);
            }
        }

        void Mutate(Solution& s)
        {
            // mutate one value for a random t,i
            int k = rnd(0, 2*SIMULATIONTURNS);
            Move& m = s[k/2][k%2];

            int h = rnd(0,1);
            Randomize(m, false);
        }

        int ComputeScore(Solution& sol, const vector<Pod>& pods)
        {
            vector<Pod> podsCopy = pods;
            sim->PlaySolution(podsCopy, sol);
            sol.score = RateSolution(podsCopy);
            return sol.score;
        }
        int RateSolution(vector<Pod>& pods)
        {
            auto podScore = [&](const Pod& p) -> int
            {
                // the max distance between 2 pods in the map (size 16k*9k) is about 18k
                // we rate each checkpoint passed higher than that:
                constexpr int cpFactor = 30000;
                const int distToCp = dist(p.position, sim->Checkpoints()[p.checkpointId]);
                return cpFactor*p.checkpointsPassed - distToCp;
            };
            for (Pod& p : pods)
                p.score = podScore(p);

            int myRacerIndex = (pods[0].score > pods[1].score) ? 0 : 1;
            const Pod& myRacer = pods[myRacerIndex];
            const Pod& myInterceptor = pods[1-myRacerIndex];

            const Pod& opponentRacer = (pods[2].score > pods[3].score) ? pods[2] : pods[3];

            if (myRacer.checkpointsPassed > sim->MaxCheckpoints())
                return INFINITY; // I won

            if (opponentRacer.checkpointsPassed > sim->MaxCheckpoints())
                return -INFINITY; // opponent won

            // how far ahead are we?
            const int aheadScore = myRacer.score - opponentRacer.score;

            // can my interceptor block the opponent racer?
            const Vector2 opponentCheckpoint = sim->Checkpoints()[opponentRacer.checkpointId];
            const int interceptorScore = -dist(myInterceptor.position, opponentCheckpoint);

            constexpr int aheadBias = 2;
            return aheadScore*aheadBias + interceptorScore;
        }
};

void OverrideAngle(Pod& p, const Vector2& target)
{
    const Vector2 dir = (target - p.position).normalized();
    float a = acos(dir.x) * 180.f / PI;
    if(dir.y<0) a = (360.f - a);
    p.angle = a;
}

int main()
{
    Simulation simulation;
    Vector2 firstCheckpoint = simulation.InitCheckpointsFromInput();

    Solver solver{ &simulation };
    vector<Pod> pods(4);

    int step=0;
    while (1)
    {
        for (int i=0; i<4; i++)
        {
            UpdatePodFromInput(pods[i], i);
            if (step == 0)
                OverrideAngle(pods[i], firstCheckpoint);
        }

        const int availableTime = (step==0) ? 500 : 75;
        const Solution& s = solver.Solve(pods, availableTime*.95f);
        ConvertSolutionToOutput(s, pods);

        ++step;
    }
}
