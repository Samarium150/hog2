#include <set>
#include <vector>
#include <list>
#include <set>
#include <map>
#include <unordered_map>
#include "Map.h"
#include <cmath>
#include <algorithm>
#include "MR1PermutationPDB.h"
#include <math.h>



template <class Env, class State>
class RASFrontier
{
public:
    Env *env;
	bool isIdle;
	unsigned int seed;
	std::vector<State> candidates;
	std::vector<State> otherCandidates;
	std::vector<State> children;
	std::vector<State> open;
	std::set<uint64_t> closed;
	std::map<uint64_t, double> gValues;
	std::map<uint64_t, State> parents;
	int numOfExp = 0;
	State rendezvous;
	RASFrontier *other;
    int sampleCount;
	State anchor;
	State start;
    Heuristic<State> *h;
	RASFrontier(Env *_env, State _start, Heuristic<State> *h, int _sampleCount);
	~RASFrontier(){}
	bool DoSingleSearchStep();
	void GetPath(std::vector<State> &path);
	void SetSeed(unsigned int seed);
	std::vector<State> GetPath(State node, bool forward);
	void ExtractPath(std::vector<State> &path);
	bool validSolution;
	bool recent_valid = false;
	//double comps = 0;

	

	double HCost(State s1, State s2)
	{
		//return abs(h->HCost(s1, s1) - h->HCost(s2, s2)) + abs(other->h->HCost(s1, s1) - other->h->HCost(s2, s2));
		return h->HCost(s1, s2); 
	}

};


template <class Env,  class State>
RASFrontier<Env, State>::RASFrontier(Env *_env, State _start, Heuristic<State> *_h, int _sampleCount)
{
    sampleCount = _sampleCount;
	env = _env;
	open.resize(0);
	h = _h;
	start = _start;
	open.push_back(start);
	gValues[env->GetStateHash(start)] = 0;
	anchor = start;
	numOfExp = 0;
	validSolution = true;
}


template <class Env,  class State>
void RASFrontier<Env, State>::ExtractPath(std::vector<State> &path)
{
	auto current = rendezvous;
	while (true)
	{
		path.push_back(current);
		if (parents.find(env->GetStateHash(current)) != parents.end())
			current = parents[env->GetStateHash(current)];
		else
			break;
	}
	path.push_back(start);
}




template <class Env,  class State>
bool RASFrontier<Env, State>::DoSingleSearchStep()
{
	//std::cout << "size: " << open.size() << std::endl;
	if (open.size() == 0)
	{
		std::cout << "******* No Solution Found! ********" << std::endl;
		return true;
	}
		
	double minDist = 99999999;
	State bestCandidate = open[0];
	int bestInd = 0;
	uint64_t bestCandidateHash = env->GetStateHash(bestCandidate);
	int _samples = min(sampleCount - 1, open.size() - 1);
	int _size = open.size();
	for (size_t i = 0; i < _samples; i++)
	{
        int index = rand() % (_size - 1);
		index++;
		State c = open[index];
		auto hash = env->GetStateHash(c);
        open[index] = open[_size - 1];
        open[_size - 1] = c;
		auto dist = HCost(c, other->anchor);
		if (dist < minDist || dist == minDist && gValues[hash] > gValues[bestCandidateHash])
		{
			minDist = dist;
			bestCandidate = c;
			bestInd = _size - 1;
			bestCandidateHash = env->GetStateHash(bestCandidate);
		}
		_size--;
	}

	if (recent_valid && HCost(open[0], other->anchor) <= minDist)
	{
		bestCandidate = open[0];
		bestInd = 0;
		bestCandidateHash = env->GetStateHash(bestCandidate);
	}
	open[bestInd] = open.back();
	open.pop_back();
	closed.insert(bestCandidateHash);
	
	numOfExp++;
	children.resize(0);
	env->GetSuccessors(bestCandidate, children);
	recent_valid = false;
	for (State neighbor : children)
	{
		auto nhash = env->GetStateHash(neighbor);
		double g = gValues[bestCandidateHash] + env->GCost(bestCandidate, neighbor);
		if (gValues.find(nhash) != gValues.end())
		{
			if (g < gValues[nhash])
			{
				gValues[nhash] = g;
				parents[nhash] = bestCandidate;
			}
		}
		else
		{
			gValues[nhash] = g;
			open.push_back(neighbor);
			parents[nhash] = bestCandidate;
			if (!recent_valid || recent_valid && HCost(neighbor, other->anchor) < HCost(open[0], other->anchor))
			{
				recent_valid = true;
				open.pop_back();
				open.push_back(open[0]);
				open[0] = neighbor;
			}
				
		}

		//if (gValues[env->GetStateHash(neighbor)] > gValues[env->GetStateHash(anchor)])
		//	anchor = neighbor;
	}
	anchor = bestCandidate;
	if (other->gValues.find(bestCandidateHash) != other->gValues.end())// || bestCandidate == other->start)
	{
		rendezvous = bestCandidate;
		other->rendezvous = rendezvous;
		return true;
	}
	return false;
}


template <class Env,  class State>
void RASFrontier<Env, State>::SetSeed(unsigned int seed)
{
	this->seed = seed;
}

template <class Env, class State>
class RAS
{
private:
	int turn = 0;
public:
	RASFrontier<Env, State>* ff;
	RASFrontier<Env, State>* bf;
	RAS();
	RAS(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount);
	~RAS(){}
	void Init(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount)
	{
		ff = new RASFrontier<Env, State>(_env, _start, hf, _sampleCount);
		bf = new RASFrontier<Env, State>(_env, _goal, hb, _sampleCount);
		ff->other = bf;
		bf->other = ff;
	}
	int GetPathLength()
	{

	}
	int GetNodesExpanded()
	{
		return ff->numOfExp + bf->numOfExp;
	}
	bool DoSingleSearchStep()
	{
		if (turn == 0)
		{
			turn = 1;
			return ff->DoSingleSearchStep();
		}
		else
		{
			turn = 0;
			return bf->DoSingleSearchStep();
		}
	}
	void ExtractPath(std::vector<State> &path)
	{
		auto current = bf->rendezvous;
		std::vector<State> front, back;
		while (true)
		{
			back.push_back(current);
			if (bf->parents.find(bf->env->GetStateHash(current)) != bf->parents.end())
				current = bf->parents[bf->env->GetStateHash(current)];
			else
				break;
		}
		back.push_back(bf->start);
		current = ff->rendezvous;
		while (true)
		{
			front.push_back(current);
			if (ff->parents.find(ff->env->GetStateHash(current)) != ff->parents.end())
				current = ff->parents[ff->env->GetStateHash(current)];
			else
				break;
		}
		front.push_back(ff->start);
		path.resize(0);
		for (int i = front.size() - 1; i >= 1; i--)
			path.push_back(front[i]);
		for (int i = 0; i < back.size(); i++)
			path.push_back(back[i]);
	}
	void GetPath(std::vector<State> &path)
	{
		path.resize(0);
		while (true)
		{
			if (DoSingleSearchStep())
			{
				break;
			}
		}
		if (!ff->validSolution || !bf->validSolution)
			return;
		ExtractPath(path);
	}
	void SetSeed(unsigned int seed)
	{
		this.seed = seed;
	}
};


template <class Env, class State>
RAS<Env, State>::RAS(Env *_env, State _start, State _goal, Heuristic<State> *hf, Heuristic<State> *hb, int _sampleCount)
{
	ff = new RASFrontier<Env, State>(_env, _start, hf, _sampleCount);
	bf = new RASFrontier<Env, State>(_env, _goal, hb, _sampleCount);
	ff->other = bf;
	bf->other = ff;
}




