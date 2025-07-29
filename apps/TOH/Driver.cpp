#include "Common.h"
#include "Driver.h"
#include "UnitSimulation.h"
#include "TOH.h"
#include "IDAStar.h"
#include "Timer.h"
#include "PermutationPDB.h"
#include "TemplateAStar.h"
#include "FileUtil.h"
#include "SVGUtil.h"
const int numDisks = 12;

void SaveSVG(Graphics::Display &d, int port);

TOH<numDisks> toh;
TOHState<numDisks> s, g;


IDAStar<TOHState<numDisks>, TOHMove> ida;
std::vector<TOHMove> solution;

TemplateAStar<TOHState<numDisks>, TOHMove, TOH<numDisks> >astar;

template <int numDisks, int pdb1Disks, int pdb2Disks = numDisks-pdb1Disks>
Heuristic<TOHState<numDisks>> *BuildPDB(const TOHState<numDisks> &goal);

// PDB info
TOHState<numDisks> goal;
TOH<numDisks-6> absToh1;
TOH<numDisks-12> absToh2;
TOH<numDisks-4> absToh3;
TOHState<numDisks-6> absTohState1;
TOHState<numDisks-12> absTohState2;
TOHState<numDisks-4> absTohState3;
TOHPDB<numDisks-6, numDisks> pdb1a(&absToh1, goal);
TOHPDB<numDisks-6, numDisks> pdb1b(&absToh1, goal);

TOHPDB<numDisks-6, numDisks> pdb1(&absToh1, goal);
TOHPDB<numDisks-12, numDisks, 12> pdb2(&absToh2, goal);
TOHPDB<numDisks-12, numDisks, 12> pdb3(&absToh2, goal);
TOHPDB<numDisks-6, numDisks, 6> pdb4(&absToh1, goal);

//TOHPDB<numDisks-4, numDisks> pdb3(&absToh3, goal);

Heuristic<TOHState<numDisks>> h;

int selectedPeg = -1; //idk
int lastClosestPeg = -1; //idk
bool recording = false;
bool animating = false;
bool dragging = false;

bool animationVersion2 = true;
float px;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 640, 640);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "PDB", "Build a PDB", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "PDB", "Value-Range Compress PDB", kAnyModifier, 'v');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Reset View", "Reset camera to initial view", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Reset state", "Choose a new random initial state", kAnyModifier, 's');
    InstallKeyboardHandler(MyDisplayHandler, "User", "Let user play", kAnyModifier, 'u');

	InstallCommandLineHandler(MyCLHandler, "-run", "-run", "Runs pre-set experiments.");
	
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
}

void SolveProblem(bool reset = true)
{
	Timer t;
	if (reset)
	{
		s.StandardStart(); //the start state
		g.Reset();         //the solution state
//		g.counts[2]++;
//		g.counts[3]--;
//		g.disks[2][0] = 4;
//		g.disks[3][3] = 3;
//		g.disks[3][4] = 2;
//		g.disks[3][5] = 1;
		//		srandom(1234);
//		for (int x = 0; x < 200; x++)
//		{
//			toh.GetActions(s, solution);
//			toh.ApplyAction(s, solution[random()%solution.size()]);
//		}
	}
	std::cout << s << "\n";
	//toh.GetStateFromHash(random()%toh.GetNumStates(s), s);
	//toh.GetStateFromHash(0, s);
//	ida.SetHeuristic(&h);
//	t.StartTimer();
//	//ida.GetPath(&toh, s, g, solution);
//	t.EndTimer();
//	for (auto &m : solution)
//	{
//		std::cout << m << " ";
//	}
//	std::cout << "\n";
//	printf("%1.2fs elapsed; %llu expanded; %llu generated [%lu]\n", t.GetElapsedTime(), ida.GetNodesExpanded(), ida.GetNodesTouched(), solution.size());

	auto *heuristic = BuildPDB<numDisks, numDisks-3>(g);

	astar.SetHeuristic(heuristic);

//	toh.pruneActions = true;
//	t.StartTimer();
//	astar.GetPath(&toh, s, g, solution);
//	t.EndTimer();
//	for (auto &m : solution)
//	{
//		std::cout << m << " ";
//	}
//	printf("\n[prune] %1.2fs elapsed; %llu expanded; %llu generated [%lu]\n", t.GetElapsedTime(), astar.GetNodesExpanded(), astar.GetNodesTouched(), solution.size());

	toh.pruneActions = false;
	t.StartTimer();
	astar.GetPath(&toh, s, g, solution);
	t.EndTimer();
	for (auto &m : solution)
	{
		std::cout << m << " ";
	}
	printf("\n[no prune] %1.2fs elapsed; %llu expanded; %llu generated [%lu]\n", t.GetElapsedTime(), astar.GetNodesExpanded(), astar.GetNodesTouched(), solution.size());

}

void SpeedTest()
{
	int nodeLimit = 10000000;
	std::vector<TOHMove> moves;
	Timer t;
	s.Reset();
	srandom(1234);
	t.StartTimer();
	for (int x = 0; x < nodeLimit; x++)
	{
		toh.GetActions(s, moves);
		toh.ApplyAction(s, moves[random()%moves.size()]);
	}
	t.EndTimer();
	printf("%1.2fs elapsed; %1.2f apply action per second\n", t.GetElapsedTime(), nodeLimit/t.GetElapsedTime());

	std::vector<TOHState<numDisks>> succ;
	s.Reset();
	t.StartTimer();
	for (int x = 0; x < nodeLimit; x++)
	{
		toh.GetSuccessors(s, succ);
		s = succ[random()%succ.size()];
	}
	t.EndTimer();
	printf("%1.2fs elapsed; %1.2f get succ per second\n", t.GetElapsedTime(), nodeLimit/t.GetElapsedTime());
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 1);
		glClearColor(0.9, 0.9, 0.9, 1.0);

		h.heuristics.push_back(&toh);
		h.lookups.push_back({kLeafNode, 0, 0});
		s.StandardStart();
		g.Reset();
//		SolveProblem(true);
//		SpeedTest();
	}
}

float v = 0.0;
uint64_t counter = 0;
const int animationFrames = 80;
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
//	cameraMoveTo(0, -1, -12.5, 0.05);
//	cameraLookAt(0, 0, 0, 0.2);
	auto &display = GetContext(windowID)->display;
	display.FillRect({-1, -1, 1, 1}, Colors::black);
//    const auto smooth = [](float a, float b, float mix)
//        { float tmp1 = mix*mix*mix; float tmp2 = (1-mix)*(1-mix)*(1-mix);
//            mix = (1-mix)*tmp1+mix*(1-tmp2); return (1-mix)*a+mix*b;
//        };

	toh.Draw(display);

   
	if (solution.size() != 0 && animating)
	{
        if (counter == 0) {
            s = g;
            toh.GetNextState(s, solution[0], g); //g is the next state
            solution.erase(solution.begin());
        }
        
//        float test = float(counter)/animationFrames;
       
        toh.Draw(display, s, g, float(counter)/animationFrames);
        counter = (counter+1)%animationFrames;
      
	}
    else if (solution.size() == 0 && animating) // is that "&& animating" part redundant?
    {
        toh.Draw(display, g); // after showing solution
    }
    else if (dragging && !animationVersion2)
    {
        
        counter = (counter+1)%animationFrames;
        v = float(counter)/animationFrames;
        toh.Draw(display, s, g, v);
    }
    else if (dragging && animationVersion2)
    {
        // have the disk teleport to the top
        // pass mouse position into Draw and have the disk follow pos.x
        // once Released, have the disk teleport to the bottom
        
        // now it needs to show the vertical animation at the beginning and at the end
        // while it's in the air, it needs to show an outline of where it's gonna drop down to/on which peg
        // we could make the peg being hovered over a diff color, and it could be animated to do a glow-fade sort of color thing
        
        toh.Draw2(display, s, selectedPeg, px);
    }
	else {
		toh.Draw(display, s); //before showing solution, or when user is playing
	}
	
	if (recording && viewport == GetNumPorts(windowID)-1)
	{
//		static int cnt = 0;
//		char fname[255];
//		sprintf(fname, "/Users/nathanst/Movies/tmp/TOH-%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
//		SaveScreenshot(windowID, fname);
//		printf("Saved %s\n", fname);
//		cnt++;
	}
	return;
	
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	exit(0);
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '|': //resetCamera(); break;
		case 'r': recording = !recording; break;
        case 'u': // y no work?
            std::cout<<"u";
            g = s;
            break;
		case 's':
		{
            std::cout<<"s";
			SolveProblem(true);
            g = s;
            animating = true;
		}
			break;
		case 'v': // value range compression
		{
			goal.Reset();
			// 268435456 entries (256 MB)
			printf("Dist: Full PDB\n");
			pdb1.BuildPDB(goal, std::thread::hardware_concurrency());
			pdb1.ValueRangeCompress(4, true);
			break;
			// 67108864 entries (64 MB) (pdb2) [72.171778]
			// 16777216 entries (16 MB) (pdb3)
			//pdb3.BuildPDB(goal, std::thread::hardware_concurrency());
	
			pdb1a = pdb1;
			pdb1b = pdb1;
			//pdb1a.DivCompress(16, true); // 16 MB
			printf("Dist: Div factor\n");
			//pdb1a.DivCompress(32, true); // 8 MB
			pdb1a.DivCompress(64, true); // 4 MB
			printf("Dist: Delta (over original)\n");
			pdb1.DeltaCompress(&pdb1a, goal, true);
			
			// delta pdb2 = 14.867143 / after VRC = 14.864208
			// delta pdb3 = 28.029138 / after VRC = 27.922551
//			pdb1.DeltaCompress(&pdb3, goal, true);
			printf("Dist: VRC\n");
			pdb1.ValueRangeCompress(2, true);

			printf("Dist: Straight 4x compression (≈2 bits)\n");
			pdb1b.DivCompress(4, true);
			printf("Dist: Straight 8x compression (≈1 bit\n");
			pdb1b.DivCompress(2, true);
			//pdb.DivCompress(4, true);
			//pdb.ModCompress(pdb.GetPDBSize()/4, true);
			h.lookups.resize(0);
			h.lookups.push_back({kMaxNode, 1, 2});
			h.lookups.push_back({kLeafNode, 0, 0});
			h.lookups.push_back({kLeafNode, 1, 1});
			h.heuristics.resize(0);
			h.heuristics.push_back(&toh);
			h.heuristics.push_back(&pdb1);
			SolveProblem();
			break;
		}
		case 'p': // div compression
		{
//			running = true;
//			recording = true;
//			break;
			goal.Reset();
			// 268435456 entries (256 MB)
			// Average: 87.038921 -> 86.762030 -> 86.483987
//			pdb1.BuildPDB(goal, std::thread::hardware_concurrency());
//			pdb2.BuildPDB(goal, std::thread::hardware_concurrency());
//			pdb3.BuildPDB(goal, std::thread::hardware_concurrency());
//			pdb4.BuildPDB(goal, std::thread::hardware_concurrency());


//			h.lookups.resize(0);
//			h.lookups.push_back({kMaxNode, 1, 4});
//			h.lookups.push_back({kLeafNode, 0, 0});
//			h.lookups.push_back({kLeafNode, 1, 1});
//			h.lookups.push_back({kLeafNode, 2, 2});
//			h.lookups.push_back({kLeafNode, 3, 3});
//			h.heuristics.resize(0);
//			h.heuristics.push_back(&pdb1);
//			h.heuristics.push_back(&pdb2);
//			h.heuristics.push_back(&pdb3);
//			h.heuristics.push_back(&pdb4);

			s.StandardStart();
			SolveProblem(false);
			//recording = true;
			break;
		}
		default:
			break;
	}
}

template <int numDisks, int pdb1Disks, int pdb2Disks>
Heuristic<TOHState<numDisks>> *BuildPDB(const TOHState<numDisks> &goal)
{
	TOH<numDisks> toh;
	TOH<pdb1Disks> absToh1;
	TOH<pdb2Disks> absToh2;
	TOHState<pdb1Disks> absTohState1;
	TOHState<pdb2Disks> absTohState2;
	
	
	TOHPDB<pdb1Disks, numDisks, pdb2Disks> *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1, goal); // top disks
	TOHPDB<pdb2Disks, numDisks> *pdb2 = new TOHPDB<pdb2Disks, numDisks>(&absToh2, goal); // bottom disks

	TOHPDB<pdb2Disks, numDisks, pdb1Disks> *pdb3 = new TOHPDB<pdb2Disks, numDisks, pdb1Disks>(&absToh2, goal); // top disks
	TOHPDB<pdb1Disks, numDisks> *pdb4 = new TOHPDB<pdb1Disks, numDisks>(&absToh1, goal); // bottom disks
	pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
	pdb2->BuildPDB(goal, std::thread::hardware_concurrency());

	pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
	pdb4->BuildPDB(goal, std::thread::hardware_concurrency());

	Heuristic<TOHState<numDisks>> *h = new Heuristic<TOHState<numDisks>>;
	
	h->lookups.resize(0);
	
	h->lookups.push_back({kMaxNode, 1, 2});
	h->lookups.push_back({kAddNode, 3, 2});
	h->lookups.push_back({kAddNode, 5, 2});
	h->lookups.push_back({kLeafNode, 0, 0});
	h->lookups.push_back({kLeafNode, 1, 1});
	h->lookups.push_back({kLeafNode, 2, 2});
	h->lookups.push_back({kLeafNode, 3, 3});
	h->heuristics.resize(0);
	h->heuristics.push_back(pdb1);
	h->heuristics.push_back(pdb2);
	h->heuristics.push_back(pdb3);
	h->heuristics.push_back(pdb4);

	return h;
}

bool MyClickHandler(unsigned long , int, int, point3d p, tButtonType , tMouseEventType e)
{
    if (e == kMouseDown)
    {
        v = 0; // ? when u click and hold on a peg, sometimes the disk animation jumps and starts towards the end of v? or is it idk
        toh.Click(s, selectedPeg, p.x);
    }
    if (e == kMouseDrag)
    {
//        dragging = toh.Drag(s, selectedPeg, p, g, v, lastClosestPeg); // reminder this is called per frame (drag drag drag drag...)
//        counter = v * 30.0f;
        px = p.x;
        dragging = toh.Drag(s, selectedPeg);
        
    }
    if (e == kMouseUp)
    {
        dragging = false;
        toh.Release(s, selectedPeg, p, g);
        
        s = g; // if g = s isn't done at the start, this makes entire stack teleport to solution
    }
    
	return true;
}

void SaveSVG(Graphics::Display &d, int port)
{
	const std::string baseFileName = "/Users/nathanst/Pictures/hog2/TOH_";
	static int count = 0;
	std::string fname;
	do {
		fname = baseFileName+std::to_string(count)+".svg";
		count++;
	} while (FileExists(fname.c_str()));
	printf("Save to '%s'\n", fname.c_str());
	MakeSVG(d, fname.c_str(), 1024, 1024, port);
}
