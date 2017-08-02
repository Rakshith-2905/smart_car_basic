def average_of_lines(line):
    """ Returns the avg of a sequence of lines. Rounds float points down to the nearest int.
     Returns an empty list if there are no lines present."""
    avg_x1 = 0
    avg_y1 = 0
    avg_x2 = 0
    avg_y2 = 0
    n_lines = len(line)
    if n_lines < 2:
        return line
    for x1,y1,x2,y2 in line:
        avg_x1 += x1
        avg_y1 += y1
        avg_x2 += x2
        avg_y2 += y2 

    return (avg_x1/ n_lines, avg_y1/ n_lines, avg_x2/ n_lines, avg_y2/ n_lines)


def intercept(line):
    """ Returns the B for a line represented by `y = mx + b` """
    # y = mx + b
    # b = y - mx
    return line.y1 - slope(line) * line.x1


def extrapolate(line, fit_y1, fit_y2):
    line_slope = slope(line)
    b = intercept(line)

    # x = (y - b) / m
    x1 = (fit_y1 - b) / line_slope
    x2 = (fit_y2 - b) / line_slope
    return Line(int(x1), int(fit_y1), int(x2), int(fit_y2))


def slope(line):
    """ Returns the slope of a line. """
    if(line!=None):
        for x1,y1,x2,y2 in line:
            rise = (y2 - y1)
            run = (x2 - x1)
        # We're trading off a tiny bit of accuracy for our program not crashing.
        if run == 0:
            run = 0.000000001
    return rise / run

