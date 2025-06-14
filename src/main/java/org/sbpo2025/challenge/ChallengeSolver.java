package org.sbpo2025.challenge;

import org.apache.commons.lang3.time.StopWatch;
import java.util.ArrayList;
import java.util.HashSet;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import ilog.concert.*;
import ilog.cplex.IloCplex;


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

        try {
            IloCplex cplex = new IloCplex();
            cplex.setOut(null); // desactiva la salida para mejorar el rendimiento

            // Variables
            IloNumVar[] x = cplex.boolVarArray(nOrders);
            IloNumVar[] y = cplex.boolVarArray(nAisles);
            IloNumVar totalUnits = cplex.intVar(waveSizeLB, waveSizeUB, "totalUnits");
            IloNumVar[] orderUnitSum = new IloNumVar[nOrders];

            for (int o = 0; o < nOrders; o++) {
                int sum = orders.get(o).values().stream().mapToInt(Integer::intValue).sum();
                orderUnitSum[o] = cplex.intVar(0, waveSizeUB, "units_o_" + o);
                cplex.addEq(orderUnitSum[o], cplex.prod(sum, x[o]));
            }

            cplex.addEq(totalUnits, cplex.sum(orderUnitSum));

            // Restricciones de capacidad por ítem
            for (int i = 0; i < nItems; i++) {
                IloLinearNumExpr expr = cplex.linearNumExpr();
                for (int o = 0; o < nOrders; o++) {
                    int units = orders.get(o).getOrDefault(i, 0);
                    if (units > 0) expr.addTerm(units, x[o]);
                }
                for (int a = 0; a < nAisles; a++) {
                    int cap = aisles.get(a).getOrDefault(i, 0);
                    if (cap > 0) expr.addTerm(-cap, y[a]);
                }
                cplex.addGe(expr, 0);
            }

            // Optimización iterativa
            double lambda = 0;
            double epsilon = 1e-4;
            double bestObj = 0;
            Set<Integer> bestOrders = new HashSet<>();
            Set<Integer> bestAisles = new HashSet<>();

            while (getRemainingTime(stopWatch) > 2) {
                IloLinearNumExpr objExpr = cplex.linearNumExpr();
                objExpr.addTerm(1.0, totalUnits);
                for (int a = 0; a < nAisles; a++) {
                    objExpr.addTerm(-lambda, y[a]);
                }
                cplex.addMaximize(objExpr);

                cplex.setParam(IloCplex.Param.TimeLimit, Math.min(getRemainingTime(stopWatch), 10));

                if (cplex.solve()) {
                    double units = cplex.getValue(totalUnits);
                    long aislesUsed = Arrays.stream(y).filter(var -> cplex.getValue(var) > 0.5).count();
                    double ratio = units / aislesUsed;

                    if (ratio - bestObj > epsilon) {
                        bestObj = ratio;
                        lambda = ratio;
                        bestOrders.clear();
                        bestAisles.clear();
                        for (int o = 0; o < nOrders; o++) if (cplex.getValue(x[o]) > 0.5) bestOrders.add(o);
                        for (int a = 0; a < nAisles; a++) if (cplex.getValue(y[a]) > 0.5) bestAisles.add(a);
                    } else {
                        break;
                    }
                } else {
                    break;
                }
                cplex.clearModel(); // importante: limpia para evitar acumulación en siguientes iteraciones
            }

            return new ChallengeSolution(bestOrders, bestAisles);
        } catch (IloException e) {
            e.printStackTrace();
            return null;
        }
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
