package org.sbpo2025.challenge;
// import com.google.ortools.Loader;
import org.apache.commons.lang3.time.StopWatch;
import java.util.ArrayList;
import java.util.HashSet;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.*;

// import com.google.ortools.linearsolver.MPSolver;
// import com.google.ortools.linearsolver.MPSolver.ResultStatus;
// import com.google.ortools.linearsolver.MPVariable;
// import com.google.ortools.linearsolver.MPConstraint;
// import com.google.ortools.linearsolver.MPObjective;
import ilog.concert.*;
import ilog.cplex.IloCplex;


public class ChallengeSolver {
    private final long MAX_RUNTIME = 600000; // milliseconds; 10 minutes

    protected List<Map<Integer, Integer>> orders;
    protected List<Map<Integer, Integer>> aisles;
    protected int nItems;
    protected int waveSizeLB;
    protected int waveSizeUB;
    static {
    Loader.loadNativeLibraries();
    }
    public ChallengeSolver(
            List<Map<Integer, Integer>> orders, List<Map<Integer, Integer>> aisles, int nItems, int waveSizeLB, int waveSizeUB) {
        this.orders = orders;
        this.aisles = aisles;
        this.nItems = nItems;
        this.waveSizeLB = waveSizeLB;
        this.waveSizeUB = waveSizeUB;
    }

    public ChallengeSolution solve(StopWatch stopWatch) {
        try {
            // 1. Crear instancia de CPLEX
            IloCplex cplex = new IloCplex();
            cplex.setParam(IloCplex.Param.TimeLimit, 599.0);                   // tiempo en segundos
            cplex.setParam(IloCplex.Param.Threads, Runtime.getRuntime().availableProcessors());

            int numOrders = orders.size();
            int numAisles = aisles.size();

            // 2. Variables binarias
            IloNumVar[] x = cplex.boolVarArray(numOrders);     // x[o] = 1 si tomo la orden o
            IloNumVar[] y = cplex.boolVarArray(numAisles);     // y[a] = 1 si visito el pasillo a

            // 2.1 Restricción: como máximo 20 pasillos
            // IloLinearNumExpr aisleCountExpr = cplex.linearNumExpr();
            // for (int a = 0; a < numAisles; a++) {
            //     aisleCountExpr.addTerm(1.0, y[a]);
            // }
            // cplex.addLe(aisleCountExpr, 20, "max_aisles");

            // 3. Precomputar unidades totales por orden
            int[] totalUnitsPerOrder = new int[numOrders];
            for (int o = 0; o < numOrders; o++) {
                totalUnitsPerOrder[o] =
                    orders.get(o).values().stream().mapToInt(Integer::intValue).sum();
            }

            // 4. Restricciones de disponibilidad por ítem
            for (int i = 0; i < nItems; i++) {
                IloLinearNumExpr expr = cplex.linearNumExpr();
                // suma de pedidos
                for (int o = 0; o < numOrders; o++) {
                    Integer q = orders.get(o).get(i);
                    if (q != null && q > 0) {
                        expr.addTerm(q, x[o]);
                    }
                }
                // menos sumatoria de pasillos
                for (int a = 0; a < numAisles; a++) {
                    Integer q = aisles.get(a).get(i);
                    if (q != null && q > 0) {
                        expr.addTerm(-q, y[a]);
                    }
                }
                cplex.addLe(expr, 0.0, "item_" + i);
            }

            // 5. Restricción de tamaño de oleada
            IloLinearNumExpr waveExpr = cplex.linearNumExpr();
            for (int o = 0; o < numOrders; o++) {
                waveExpr.addTerm(totalUnitsPerOrder[o], x[o]);
            }
            cplex.addRange(waveSizeLB, waveExpr, waveSizeUB, "wave_size");

            // 6. Objetivo: maximizar totalUnits*bigM - numPasillos
            IloLinearNumExpr objExpr = cplex.linearNumExpr();
            int bigM = 1_500;
            for (int o = 0; o < numOrders; o++) {
                objExpr.addTerm(totalUnitsPerOrder[o] * bigM, x[o]);
            }
            for (int a = 0; a < numAisles; a++) {
                objExpr.addTerm(-1.0, y[a]);
            }
            cplex.addMaximize(objExpr);

            // 7. Resolver
            if (cplex.solve()) {
                Set<Integer> selectedOrders = new HashSet<>();
                Set<Integer> visitedAisles = new HashSet<>();
                for (int o = 0; o < numOrders; o++) {
                    if (cplex.getValue(x[o]) > 0.5) {
                        selectedOrders.add(o);
                    }
                }
                for (int a = 0; a < numAisles; a++) {
                    if (cplex.getValue(y[a]) > 0.5) {
                        visitedAisles.add(a);
                    }
                }
                cplex.end();
                return new ChallengeSolution(selectedOrders, visitedAisles);
            } else {
                cplex.end();
                // Ninguna solución factible encontrada
                return new ChallengeSolution(Collections.emptySet(), Collections.emptySet());
            }
        } catch (IloException e) {
            throw new RuntimeException("Error en CPLEX: " + e.getMessage(), e);
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
