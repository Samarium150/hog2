#include "Driver.h"
#include "SolutionUtil.h"

std::vector <RegionConstraintItem> gRegionConstraintItems = {
    {{kSeparation,      0,  Colors::cyan}, Graphics::point{-0.75, -0.7}, 0.1},
    {{kStar,           0,  Colors::cyan}, Graphics::point{-0.5, -0.7},  0.1},
    {{kTetris,         10, Colors::cyan}, Graphics::point{-0.25, -0.7}, 0.05},
    {{kNegativeTetris,  10, Colors::cyan}, Graphics::point{-0.05, -0.7}, 0.05},
    {{kTriangle,       1,  Colors::cyan}, Graphics::point{-0.75, -0.5}, 0.05},
    {{kTriangle,       2,  Colors::cyan}, Graphics::point{-0.5, -0.5},  0.05},
    {{kTriangle,       3,  Colors::cyan}, Graphics::point{-0.2, -0.5},  0.05},
};

std::vector <PathConstraintItem> gPathConstraintItems = {
    {kNoPathConstraint, Graphics::point{0.1, -0.5}, 0.075},
    {kMustCross,    Graphics::point{0.3, -0.5}, 0.075},
    {kCannotCross,  Graphics::point{0.5, -0.5}, 0.075}
};

int gSelectedEditorItem = -1;
unsigned gSelectedTetrisItem = 0;
unsigned gSelectedColor = 0;
static Graphics::point gLastPosition = Graphics::point{};
std::vector<size_t> currentSolutionIndices;
size_t gNumSolutions = 0;

std::vector <ColorItem> gProvidedColors = {
    {Colors::red,   Graphics::point{-0.75, -0.15}, 0.1},
    {Colors::orange,     Graphics::point{-0.6, -0.15},  0.1},
    {{0.862745098f, 0.6549019608f, 0.0f},   Graphics::point{-0.45, -0.15}, 0.1},
    {Colors::blue,    Graphics::point{-0.3, -0.15},  0.1},
    {Colors::cyan,  Graphics::point{-0.15, -0.15}, 0.1},
    {Colors::magenta,    Graphics::point{0.0, -0.15},   0.1},
    {Colors::pink, Graphics::point{0.15, -0.15},  0.1},
    {Colors::black,   Graphics::point{0.3, -0.15},   0.1},
};

Witness <puzzleWidth, puzzleHeight> editor;

static size_t GetNumValidSolutions(bool isAdding) {
    size_t ret = 0;
    if (isAdding)
    {
        for (const size_t &i : currentSolutionIndices)
        {
            if (editor.GoalTest(allSolutions[i]))
                ++ret;
        }
    }
    else
    {
        for (const auto &solution : allSolutions)
        {
            if (editor.GoalTest(solution))
                ++ret;
        }
    }
    return ret;
}

static void DrawGameViewport(unsigned long windowID)
{
    Graphics::Display &display = GetContext(windowID)->display;
    witness.Draw(display);
    if (!drawEditor)
    {
        iws.IncrementTime();
        witness.Draw(display, iws);
        if (solved)
        {
            display.DrawText("Solved!", Graphics::point{1, 1}, Colors::black, 0.075,
                             Graphics::textAlignRight, Graphics::textBaselineBottom);
        }
    }
    else
    {
        display.DrawText("# of solutions: ", Graphics::point{0.75, 1}, Colors::black, 0.075,
                         Graphics::textAlignRight, Graphics::textBaselineBottom);
        if (gSelectedEditorItem != -1 && cursorViewport == 0)
        {
            if (gSelectedEditorItem < gRegionConstraintItems.size())
            {
                bool cursorInPuzzle = false;
                WitnessRegionConstraint constraint = gRegionConstraintItems[gSelectedEditorItem].constraint;
                for (unsigned i = 0; i < witness.regionConstraintLocations.size(); ++i)
                {
                    const auto &location = witness.regionConstraintLocations[i];
                    Graphics::point p = location.first;
                    if (PointInRect(cursor, location.second))
                    {
                        unsigned y = i % puzzleWidth;
                        unsigned x = (i - y) / puzzleWidth;
                        if (p != gLastPosition) {
                            bool isAdding;
                            if (constraint == witness.regionConstraints[x][y])
                            {
                                editor.ClearConstraint(x, y);
                                isAdding = false;
                            }
                            else
                            {
                                switch (constraint.t) {
                                    case kSeparation:
                                        editor.AddSeparationConstraint(x, y, constraint.c);
                                        break;
                                    case kStar:
                                        editor.AddStarConstraint(x, y, constraint.c);
                                        break;
                                    case kTetris:
                                        editor.AddTetrisConstraint(x, y, constraint.parameter);
                                        break;
                                    case kNegativeTetris:
                                        editor.AddNegativeTetrisConstraint(x, y, constraint.parameter);
                                        break;
                                    case kTriangle:
                                        editor.AddTriangleConstraint(x, y, constraint.parameter);
                                        break;
                                    case kEraser:
                                        break;
                                    default: // kNoRegionConstraint, kRegionConstraintCount
                                        break;
                                }
                                isAdding = true;
                            }
                            gNumSolutions = GetNumValidSolutions(isAdding);
                        }
                        display.FrameRect(location.second, (gNumSolutions > 0) ? Colors::gray : Colors::red, 0.01);
                        editor.DrawRegionConstraint(display, constraint, p);
                        gLastPosition = p;
                        cursorInPuzzle = true;
                        break;
                    }
                }
                if (!cursorInPuzzle)
                    witness.DrawRegionConstraint(display, constraint, cursor);
            }
            else
            {
                for (unsigned i = 0; i < witness.pathConstraintLocations.size(); ++i)
                {
                    const auto &location = witness.pathConstraintLocations[i];
                    if (PointInRect(cursor, location.second) &&
                        i != puzzleWidth * (puzzleHeight + 1) + (puzzleWidth + 1) * puzzleHeight)
                    {
                        WitnessPathConstraintType constraint =
                                gPathConstraintItems[gSelectedEditorItem - gRegionConstraintItems.size()]
                                        .constraint;
                        if (location.first != gLastPosition) {
                            bool isAdding = false;
                            if (constraint == witness.pathConstraints[i])
                                editor.pathConstraints[i] = kNoPathConstraint;
                            else
                            {
                                editor.pathConstraints[i] = constraint;
                                if (constraint != kNoPathConstraint)
                                    isAdding = true;
                            }
                            gNumSolutions = GetNumValidSolutions(isAdding);
                        }
                        display.FrameRect(location.second, (gNumSolutions > 0) ? Colors::gray : Colors::red, 0.01);
                        gLastPosition = location.first;
                        break;
                    }
                }
            }
            display.DrawText(std::to_string(gNumSolutions).c_str(), Graphics::point{0.9, 1}, Colors::black, 0.075,
                             Graphics::textAlignRight, Graphics::textBaselineBottom);
        }
        else
        {
            display.DrawText(std::to_string(currentSolutionIndices.size()).c_str(), Graphics::point{0.9, 1},
                             Colors::black, 0.075, Graphics::textAlignRight, Graphics::textBaselineBottom);
        }
    }
}

static void DrawEditorViewport(unsigned long windowID)
{
    if (!drawEditor) return;
    Graphics::Display &display = GetContext(windowID)->display;

    display.FillRect({-1.0f, -1.0f, 1.0f, 1.0f}, Colors::gray);

    display.DrawText("Select a constraint", Graphics::point{-0.8, -0.83}, Colors::black, 0.05);
    for (unsigned i = 0; i < gRegionConstraintItems.size(); ++i)
    {
        RegionConstraintItem &item = gRegionConstraintItems[i];
        editor.DrawRegionConstraint(display, item.constraint, item.c);
        if (i == gSelectedEditorItem)
        {
            if (i < gRegionConstraintItems.size() - 2)
                display.FrameRect({item.c, item.radius + 0.01f}, Colors::white, 0.01);
            else if (i == gRegionConstraintItems.size() - 2)
                display.FrameRect({item.c.x - item.radius - 0.03f, item.c.y - item.radius - 0.01f,
                                   item.c.x + item.radius + 0.03f, item.c.y + item.radius + 0.01f},
                                  Colors::white, 0.01);
            else
                display.FrameRect({item.c.x - item.radius - 0.06f, item.c.y - item.radius - 0.01f,
                                   item.c.x + item.radius + 0.06f, item.c.y + item.radius + 0.01f},
                                  Colors::white, 0.01);
        }
        else if (cursorViewport == 1 && PointInRect(cursor, {item.c, item.radius + 0.01f}))
        {
            if (i < gRegionConstraintItems.size() - 2)
                display.FrameRect({item.c, item.radius + 0.01f}, Colors::lightgray, 0.01);
            else if (i == gRegionConstraintItems.size() - 2)
                display.FrameRect({item.c.x - item.radius - 0.03f, item.c.y - item.radius - 0.01f,
                                   item.c.x + item.radius + 0.03f, item.c.y + item.radius + 0.01f},
                                  Colors::lightgray, 0.01);
            else
                display.FrameRect({item.c.x - item.radius - 0.06f, item.c.y - item.radius - 0.01f,
                                   item.c.x + item.radius + 0.06f, item.c.y + item.radius + 0.01f},
                                  Colors::lightgray, 0.01);
        }
    }
    for (unsigned i = 0; i < gPathConstraintItems.size(); ++i)
    {
        PathConstraintItem &item = gPathConstraintItems[i];
        if (i == gSelectedEditorItem - gRegionConstraintItems.size())
            display.FillRect({item.c, item.radius + 0.01f}, Colors::green);
        else if (cursorViewport == 1 && PointInRect(cursor, {item.c, item.radius + 0.01f}))
            display.FillRect({item.c, item.radius + 0.01f}, Colors::lightgray);
        else
            display.FillRect({item.c, item.radius + 0.01f}, Colors::white);
        display.FillRect({item.c.x - item.radius, item.c.y - 0.025f, item.c.x + item.radius, item.c.y + 0.025f},
                         witness.drawColor);
        if (item.constraint == kCannotCross)
            display.FillRect({item.c, 0.025}, witness.backColor);
        else if (item.constraint == kMustCross)
            display.FillNGon(item.c, 0.025, 6, 30, witness.backColor);
    }
    display.DrawText("Select a color", Graphics::point{-0.8, -0.25}, Colors::black, 0.05);
    for (const auto &item: gProvidedColors)
    {
        Graphics::rect rc = {item.c, 0.05f};
        display.FillRect(rc, item.color);
        if (cursorViewport == 1 && PointInRect(cursor, rc))
            display.FrameRect(rc, Colors::lightgray, 0.01);
    }
    display.DrawText("Clear All", Graphics::point{0.5, -0.15}, Colors::black, 0.05);
    Graphics::rect rc = {0.49, -0.22, 0.7, -0.14};
    if (cursorViewport == 1 && PointInRect(cursor, rc))
        display.FrameRect(rc, Colors::lightgray, 0.01);
}

static void DrawTetrisPiecesViewport(unsigned long windowID)
{
    if (!drawEditor) return;
    Graphics::Display &display = GetContext(windowID)->display;
    display.FillRect({-1.0f, -1.0f, 1.0f, 1.0f}, Colors::bluegray);
    //    printf("selected tetris piece: %d\n", selectTetrisPiece);
    for (const auto &item: gTetrisPieces)
    {
        WitnessRegionConstraintType t = (selectTetrisPiece == 2) ? kNegativeTetris : kTetris;
        WitnessRegionConstraint constraint = {.t = t, .parameter = item.parameter, .c = Colors::white};
        editor.DrawRegionConstraint(display, constraint, item.c);
        if (cursorViewport == 2 && PointInRect(cursor, {item.c, item.radius})) {
            display.FrameRect({item.c, item.radius}, Colors::lightgray, 0.01);
        }
    }
}

void WitnessFrameHandler(unsigned long windowID, unsigned int viewport, void * /*data*/)
{
    switch (viewport)
    {
        case 0:
            DrawGameViewport(windowID);
            break;
        case 1:
        {
            DrawEditorViewport(windowID);
            editor = witness;
            break;
        }
        case 2:
        {
            DrawTetrisPiecesViewport(windowID);
            break;
        }
        default:
            break;
    }
}
