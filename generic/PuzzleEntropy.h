//
//  PuzzleEntropy.hpp
//  The Witness Editor
//
//  Created by Samarium on 2023-07-21.
//  Copyright © 2023 MovingAI. All rights reserved.
//
#ifndef HOG2_GENERIC_PUZZLE_ENTROPY_H
#define HOG2_GENERIC_PUZZLE_ENTROPY_H

#include <numeric>
#include <vector>
#include "PuzzleInferenceRule.h"
#include "SearchEnvironment.h"
#include "vectorCache.h"

struct EntropyInfo
{
    double value;
    unsigned depth;
};

template<class State, class Action>
class Entropy
{
protected:
    static constexpr double inf = std::numeric_limits<double>::max();
    vectorCache<Action> actCache;
    vectorCache<EntropyInfo> entropyInfoCache;
    vectorCache<double> doubleCache;
    bool isRelative = false;

    static void Softmin(const std::vector<double> &vars, std::vector<double> &ret)
    {
        double sum = std::accumulate(vars.begin(), vars.end(), 0.0, [](double r, double i) {
            return r + std::exp(-i);
        });
        ret.reserve(vars.size());
        std::transform(vars.begin(), vars.end(), std::back_inserter(ret), [&](double i) {
            return std::exp(-i) / sum;
        });
    }

    static double KlDivergence(const std::vector<double> &p, const std::vector<double> &q)
    {
        if (p.size() != q.size())
            throw std::invalid_argument("Input vectors must be of the same size.");
        double divergence = 0.0;
        for (size_t i = 0; i < p.size(); ++i)
        {
            if (p[i] != 0.0 && q[i] != 0.0)
                divergence += p[i] * std::log2(p[i] / q[i]);
        }
        return divergence;
    }

    double ImmediateEntropy(const std::vector<Action> &actions,
                            const std::vector<double> &childEntropy,
                            std::optional<Action> prevAction)
    {
        auto size = actions.size();
        if (!isRelative)
            return std::log2(static_cast<double>(size));
        std::vector<double> &childDist = *doubleCache.getItem();
        Softmin(childEntropy, childDist);
        std::vector<double> &expectedDist = *doubleCache.getItem();
        expectedDist.resize(size);
        std::generate(expectedDist.begin(), expectedDist.end(), [
            size = static_cast<double>(size)
        ]() { return 1 / size; });
        if (size > 1 && prevAction.has_value() &&
            std::find(actions.begin(), actions.end(), prevAction.value()) != actions.end())
        {
            for (auto i = 0; i < size; ++i)
            {
                if (actions[i] == prevAction.value())
                    expectedDist[i] += probShift;
                else
                    expectedDist[i] -= probShift / (size - 1);
            }
        }
        auto kl = KlDivergence(childDist, expectedDist);
        doubleCache.returnItem(&childDist);
        doubleCache.returnItem(&expectedDist);
        return kl;
    }

public:
    PuzzleInferenceRuleSet<State, Action> ruleSet;
    double probShift = 0.0;

    auto& SetRelative(bool val)
    {
        this->isRelative = val;
        return *this;
    }
    
    auto& SetShift(double val)
    {
        this->probShift = val;
        return *this;
    }

    virtual EntropyInfo Calculate(const SearchEnvironment<State, Action> &env, State &state, unsigned lookahead, 
                                  std::optional<Action> prevAction = {})
    {
        if (env.GoalTest(state))
            return { 0.0, 0 };
        std::vector<Action> &allActions = *actCache.getItem();
        env.GetActions(state, allActions);
        ruleSet.FilterActions(env, state, allActions);
        if (allActions.empty())
        {
            actCache.returnItem(&allActions);
            return { inf, 0 };
        }
        std::vector<EntropyInfo> &childEntropyInfo = *entropyInfoCache.getItem();
        for (const auto &action: allActions)
        {
            env.ApplyAction(state, action);
            childEntropyInfo.emplace_back(Calculate(env, state, (lookahead > 0) ? lookahead - 1 : 0, action));
            env.UndoAction(state, action);
        }
        for (auto it = childEntropyInfo.begin(); it != childEntropyInfo.end(); )
        {
            auto &info = *it;
            if (info.value == 0 && info.depth < lookahead)
                return { 0.0, info.depth + 1 };
            if (info.value == inf && info.depth < lookahead)
                it = childEntropyInfo.erase(it);
            else
                ++it;
        }
        EntropyInfo entropyInfo;
        if (std::all_of(childEntropyInfo.begin(), childEntropyInfo.end(), [](EntropyInfo &info) {
            return info.value == 0;
        }))
            entropyInfo = { 0.0, childEntropyInfo[0].depth + 1 };
        else if (std::all_of(childEntropyInfo.begin(), childEntropyInfo.end(), [](EntropyInfo &info) {
            return info.value == inf;
        }))
            entropyInfo = { inf, childEntropyInfo[0].depth + 1 };
        else
        {
            auto &min_childEntropyInfo = *std::min_element(childEntropyInfo.begin(), childEntropyInfo.end(),
                                       [](EntropyInfo &info1, EntropyInfo &info2){
                return info1.value < info2.value;
            });
            auto childEntropy = std::vector<double>();
            childEntropy.reserve(childEntropyInfo.size());
            std::transform(childEntropyInfo.begin(), childEntropyInfo.end(), std::back_inserter(childEntropy),
                           [](EntropyInfo &info){ return info.value; });
            entropyInfo = { min_childEntropyInfo.value + ImmediateEntropy(allActions, childEntropy, prevAction),
                            min_childEntropyInfo.depth + 1 };
        }
        entropyInfoCache.returnItem(&childEntropyInfo);
        actCache.returnItem(&allActions);
        return entropyInfo;
    }
};

#endif /* HOG2_GENERIC_PUZZLE_ENTROPY_H */
