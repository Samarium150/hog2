#include "../Driver.h"

std::vector <RegionConstraintItem> gRegionConstraintItems = {
    {{kRegion,         0,  Colors::cyan}, Graphics::point{-0.75, -0.7}, 0.1},
    {{kStar,           0,  Colors::cyan}, Graphics::point{-0.5, -0.7},  0.1},
    {{kTetris,         10, Colors::cyan}, Graphics::point{-0.25, -0.7}, 0.05},
    {{kNegativeTetris, 10, Colors::cyan}, Graphics::point{-0.05, -0.7}, 0.05},
    {{kTriangle,       1,  Colors::cyan}, Graphics::point{-0.75, -0.5}, 0.05},
    {{kTriangle,       2,  Colors::cyan}, Graphics::point{-0.5, -0.5},  0.05},
    {{kTriangle,       3,  Colors::cyan}, Graphics::point{-0.2, -0.5},  0.05},
};

std::vector <PathConstraintItem> gPathConstraintItems = {{kNoConstraint, Graphics::point{0.1, -0.5}, 0.075},
    {kMustCross,    Graphics::point{0.3, -0.5}, 0.075},
    {kCannotCross,  Graphics::point{0.5, -0.5}, 0.075}};

int gSelectedEditorItem = -1;
unsigned gSelectedTetrisItem = 0;
unsigned gSelectedColor = 0;

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
                for (unsigned i = 0; i < witness.regionConstraintLocations.size(); ++i)
                {
                    auto &location = witness.regionConstraintLocations[i];
                    Graphics::point p = location.first;
                    if (PointInRect(cursor, location.second))
                    {
                        unsigned y = i % puzzleWidth;
                        unsigned x = (i - y) / puzzleWidth;
                        display.FrameRect(location.second, Colors::gray, 0.01);
                        WitnessRegionConstraint constraint = gRegionConstraintItems[gSelectedEditorItem].constraint;
                        witness.DrawRegionConstraint(display, constraint, p);
                        if (constraint == witness.regionConstraints[x][y])
                            editor.regionConstraints[x][y] = {.t = kNone, .parameter = 0, .c = Colors::white};
                        else
                            editor.regionConstraints[x][y] = constraint;
                        std::vector <WitnessState<puzzleWidth, puzzleHeight>> allSolutions;
                        GetAllSolutions(editor, allSolutions);
                        size_t solutions = allSolutions.size();
                        display.DrawText(std::to_string(solutions).c_str(), Graphics::point{0.9, 1}, Colors::black, 0.075,
                                         Graphics::textAlignRight, Graphics::textBaselineBottom);
                    }
                }
            }
            else
            {
                for (unsigned i = 0; i < witness.pathConstraintLocations.size() - 1; ++i)
                {
                    if (PointInRect(cursor, witness.pathConstraintLocations[i].second) &&
                        i != puzzleWidth * (puzzleHeight + 1) + (puzzleWidth + 1) * puzzleHeight)
                    {
                        display.FrameRect(witness.pathConstraintLocations[i].second, Colors::gray, 0.01);
                    }
                }
            }
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
                display.FrameRect({item.c.x - item.radius - 0.03f, item.c.y - item.radius - 0.01f, item.c.x + item.radius + 0.03f, item.c.y + item.radius + 0.01f}, Colors::white, 0.01);
            else
                display.FrameRect({item.c.x - item.radius - 0.06f, item.c.y - item.radius - 0.01f, item.c.x + item.radius + 0.06f, item.c.y + item.radius + 0.01f}, Colors::white, 0.01);
        }
        else if (cursorViewport == 1 && PointInRect(cursor, {item.c, item.radius + 0.01f}))
        {
            if (i < gRegionConstraintItems.size() - 2)
                display.FrameRect({item.c, item.radius + 0.01f}, Colors::lightgray, 0.01);
            else if (i == gRegionConstraintItems.size() - 2)
                display.FrameRect({item.c.x - item.radius - 0.03f, item.c.y - item.radius - 0.01f, item.c.x + item.radius + 0.03f, item.c.y + item.radius + 0.01f}, Colors::lightgray, 0.01);
            else
                display.FrameRect({item.c.x - item.radius - 0.06f, item.c.y - item.radius - 0.01f, item.c.x + item.radius + 0.06f, item.c.y + item.radius + 0.01f}, Colors::lightgray, 0.01);
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
    Graphics::Display &display = GetContext(windowID)->display;
    if (gSelectedEditorItem != -1 && viewport == cursorViewport)
    {
        if (gSelectedEditorItem < gRegionConstraintItems.size())
        {
            if (gSelectedEditorItem == 2 || gSelectedEditorItem == 3)
            {
                if (gSelectedTetrisItem != 0)
                    witness.DrawRegionConstraint(display, gRegionConstraintItems[gSelectedEditorItem].constraint, cursor);
            }
            else
                witness.DrawRegionConstraint(display, gRegionConstraintItems[gSelectedEditorItem].constraint, cursor);
        }
    }
}
