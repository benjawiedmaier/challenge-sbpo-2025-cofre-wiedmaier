package org.sbpo2025.challenge;

import org.apache.commons.lang3.time.StopWatch;
import java.util.ArrayList;
import java.util.HashSet;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPVariable;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPConstraint;


public class ChallengeSolver {
    private final long MAX_RUNTIME = 600000; // milliseconds; 10 minutes

    protected List<Map<Integer, Integer>> orders;
    protected List<Map<Integer, Integer>> aisles;
    protected int nItems;
    protected int waveSizeLB;
    protected int waveSizeUB;

    public ChallengeSolver(
            List<Map<Integer, Integer>> orders, List<Map<Integer, Integer>> aisles, int nItems, int waveSizeLB, int waveSizeUB) {
        this.orders = orders;
        this.aisles = aisles;
        this.nItems = nItems;
        this.waveSizeLB = waveSizeLB;
        this.waveSizeUB = waveSizeUB;
    }

    public ChallengeSolution solve(StopWatch stopWatch) {
        int nOrders = orders.size();
        int nAisles = aisles.size();

        MPSolver solver = MPSolver.createSolver("CBC");
        if (solver == null) return null;

        MPVariable[] x = new MPVariable[nOrders];
        for (int o = 0; o < nOrders; o++) {
            x[o] = solver.makeBoolVar("x_o_" + o);
        }

        MPVariable[] y = new MPVariable[nAisles];
        for (int a = 0; a < nAisles; a++) {
            y[a] = solver.makeBoolVar("y_a_" + a);
        }

        MPVariable totalUnits = solver.makeIntVar(waveSizeLB, waveSizeUB, "total_units");

        MPVariable[] orderUnitSum = new MPVariable[nOrders];
        for (int o = 0; o < nOrders; o++) {
            int sum = orders.get(o).values().stream().mapToInt(Integer::intValue).sum();
            orderUnitSum[o] = solver.makeIntVar(0, waveSizeUB, "units_o_" + o);
            MPConstraint c = solver.makeConstraint(0.0, 0.0, "link_units_o_" + o);
            c.setCoefficient(orderUnitSum[o], 1.0);
            c.setCoefficient(x[o], -sum);
        }

        MPConstraint totalUnitsConstraint = solver.makeConstraint(0.0, 0.0, "total_units_def");
        totalUnitsConstraint.setCoefficient(totalUnits, 1.0);
        for (MPVariable v : orderUnitSum) {
            totalUnitsConstraint.setCoefficient(v, -1.0);
        }

        for (int i = 0; i < nItems; i++) {
            MPConstraint itemConstraint = solver.makeConstraint(0.0, Double.POSITIVE_INFINITY, "item_req_supply_" + i);
            for (int o = 0; o < nOrders; o++) {
                int units = orders.get(o).getOrDefault(i, 0);
                if (units > 0) {
                    itemConstraint.setCoefficient(x[o], units);
                }
            }
            for (int a = 0; a < nAisles; a++) {
                int capacity = aisles.get(a).getOrDefault(i, 0);
                if (capacity > 0) {
                    itemConstraint.setCoefficient(y[a], -capacity);
                }
            }
        }

        double lambda = 0;
        double epsilon = 1e-4;
        double bestObj = 0;
        Set<Integer> bestOrders = new HashSet<>();
        Set<Integer> bestAisles = new HashSet<>();

        while (getRemainingTime(stopWatch) > 2) {
            MPObjective objective = solver.objective();
            objective.setMaximization();
            objective.setCoefficient(totalUnits, 1.0);
            for (int a = 0; a < nAisles; a++) {
                objective.setCoefficient(y[a], -lambda);
            }

            solver.setTimeLimit((int)Math.min(getRemainingTime(stopWatch) * 1000, 10000));
            final MPSolver.ResultStatus resultStatus = solver.solve();

            if (resultStatus == MPSolver.ResultStatus.OPTIMAL || resultStatus == MPSolver.ResultStatus.FEASIBLE) {
                double units = totalUnits.solutionValue();
                double aislesUsed = Arrays.stream(y).filter(var -> var.solutionValue() > 0.5).count();
                double ratio = units / aislesUsed;

                if (ratio - bestObj > epsilon) {
                    bestObj = ratio;
                    lambda = ratio;
                    bestOrders.clear();
                    bestAisles.clear();
                    for (int o = 0; o < nOrders; o++) if (x[o].solutionValue() > 0.5) bestOrders.add(o);
                    for (int a = 0; a < nAisles; a++) if (y[a].solutionValue() > 0.5) bestAisles.add(a);
                } else {
                    break;
                }
            } else {
                break;
            }
        }

        return new ChallengeSolution(bestOrders, bestAisles);
    }
    protected long getRemainingTime(StopWatch stopWatch) {
        return Math.max(
                TimeUnit.SECONDS.convert(MAX_RUNTIME - stopWatch.getTime(TimeUnit.MILLISECONDS), TimeUnit.MILLISECONDS),
                0);
    }

    protected boolean isSolutionFeasible(ChallengeSolution challengeSolution) {
        Set<Integer> selectedOrders = challengeSolution.orders();
        Set<Integer> visitedAisles = challengeSolution.aisles();
        if (selectedOrders == null || visitedAisles == null || selectedOrders.isEmpty() || visitedAisles.isEmpty()) {
            return false;
        }

        int[] totalUnitsPicked = new int[nItems];
        int[] totalUnitsAvailable = new int[nItems];

        for (int order : selectedOrders) {
            for (Map.Entry<Integer, Integer> entry : orders.get(order).entrySet()) {
                totalUnitsPicked[entry.getKey()] += entry.getValue();
            }
        }

        for (int aisle : visitedAisles) {
            for (Map.Entry<Integer, Integer> entry : aisles.get(aisle).entrySet()) {
                totalUnitsAvailable[entry.getKey()] += entry.getValue();
            }
        }

        int totalUnits = Arrays.stream(totalUnitsPicked).sum();
        if (totalUnits < waveSizeLB || totalUnits > waveSizeUB) {
            return false;
        }

        for (int i = 0; i < nItems; i++) {
            if (totalUnitsPicked[i] > totalUnitsAvailable[i]) {
                return false;
            }
        }

        return true;
    }

    protected double computeObjectiveFunction(ChallengeSolution challengeSolution) {
        Set<Integer> selectedOrders = challengeSolution.orders();
        Set<Integer> visitedAisles = challengeSolution.aisles();
        if (selectedOrders == null || visitedAisles == null || selectedOrders.isEmpty() || visitedAisles.isEmpty()) {
            return 0.0;
        }
        int totalUnitsPicked = 0;

        for (int order : selectedOrders) {
            totalUnitsPicked += orders.get(order).values().stream()
                    .mapToInt(Integer::intValue)
                    .sum();
        }

        int numVisitedAisles = visitedAisles.size();

        return (double) totalUnitsPicked / numVisitedAisles;
    }
}
