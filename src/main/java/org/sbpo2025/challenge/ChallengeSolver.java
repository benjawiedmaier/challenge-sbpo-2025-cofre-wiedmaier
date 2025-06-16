package org.sbpo2025.challenge;
import com.google.ortools.Loader;
import org.apache.commons.lang3.time.StopWatch;
import java.util.ArrayList;
import java.util.HashSet;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.*;

import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPSolver.ResultStatus;
import com.google.ortools.linearsolver.MPVariable;
import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;

// import com.google.ortools.sat.CpModel;
import com.google.ortools.sat.CpModel;
import com.google.ortools.sat.CpSolver;
import com.google.ortools.sat.CpSolverStatus;
import com.google.ortools.sat.BoolVar;
import com.google.ortools.sat.LinearExpr;

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
        // 2. Crea el modelo CP-SAT
        CpModel model = new CpModel();

        int numOrders = orders.size();
        int numAisles = aisles.size();

        // 3. Variables booleanas
        BoolVar[] x = new BoolVar[numOrders];
        BoolVar[] y = new BoolVar[numAisles];
        for (int o = 0; o < numOrders; o++) {
            x[o] = model.newBoolVar("x_o_" + o);
        }
        for (int a = 0; a < numAisles; a++) {
            y[a] = model.newBoolVar("y_a_" + a);
        }

        // 4. Precomputar unidades totales por orden
        int[] totalUnitsPerOrder = new int[numOrders];
        for (int o = 0; o < numOrders; o++) {
            totalUnitsPerOrder[o] =
                orders.get(o).values().stream().mapToInt(Integer::intValue).sum();
        }

        // 5. Restricción de tamaño de la oleada
        {
            LinearExpr.Builder waveExpr = LinearExpr.newBuilder();
            for (int o = 0; o < numOrders; o++) {
                waveExpr.addTerm(x[o], totalUnitsPerOrder[o]);
            }
            model.addLinearConstraint(waveExpr, waveSizeLB, waveSizeUB);
        }

        // 6. Restricciones de disponibilidad por ítem: demand ≤ supply
        for (int i = 0; i < nItems; i++) {
            LinearExpr.Builder demand = LinearExpr.newBuilder();
            LinearExpr.Builder supply = LinearExpr.newBuilder();
            for (int o = 0; o < numOrders; o++) {
                Integer q = orders.get(o).get(i);
                if (q != null && q > 0) {
                    demand.addTerm(x[o], q);
                }
            }
            for (int a = 0; a < numAisles; a++) {
                Integer q = aisles.get(a).get(i);
                if (q != null && q > 0) {
                    supply.addTerm(y[a], q);
                }
            }
            model.addLessOrEqual(demand, supply);
        }

        // 7. Función objetivo: max bigM·(unidades recogidas) – (nº pasillos)
        {
            LinearExpr.Builder obj = LinearExpr.newBuilder();
            int bigM = 1_000;
            for (int o = 0; o < numOrders; o++) {
                obj.addTerm(x[o], totalUnitsPerOrder[o] * bigM);
            }
            for (int a = 0; a < numAisles; a++) {
                obj.addTerm(y[a], -1);
            }
            model.maximize(obj);
        }

        // 8. Resolver con límite de tiempo
        CpSolver solver = new CpSolver();
        solver.getParameters().setMaxTimeInSeconds(MAX_RUNTIME / 1000.0);
        CpSolverStatus status = solver.solve(model);

        // 9. Extraer la solución
        Set<Integer> selectedOrders = new HashSet<>();
        Set<Integer> visitedAisles = new HashSet<>();
        if (status == CpSolverStatus.OPTIMAL || status == CpSolverStatus.FEASIBLE) {
            for (int o = 0; o < numOrders; o++) {
                if (solver.booleanValue(x[o])) {
                    selectedOrders.add(o);
                }
            }
            for (int a = 0; a < numAisles; a++) {
                if (solver.booleanValue(y[a])) {
                    visitedAisles.add(a);
                }
            }
        }

        return new ChallengeSolution(selectedOrders, visitedAisles);
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
