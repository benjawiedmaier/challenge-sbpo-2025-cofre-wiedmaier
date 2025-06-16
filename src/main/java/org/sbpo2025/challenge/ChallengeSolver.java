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
        // 1. Carga de librerías nativas de OR-Tools
        Loader.loadNativeLibraries();

        // 2. Construcción del modelo CP-SAT
        CpModel model = new CpModel();
        int numOrders = orders.size();
        int numAisles = aisles.size();

        // 3. Variables booleanas
        BoolVar[] x = new BoolVar[numOrders];
        for (int o = 0; o < numOrders; o++) {
            x[o] = model.newBoolVar("x_o_" + o);
        }
        BoolVar[] y = new BoolVar[numAisles];
        for (int a = 0; a < numAisles; a++) {
            y[a] = model.newBoolVar("y_a_" + a);
        }

        // 4. Precomputar unidades totales por orden
        int[] totalUnitsPerOrder = new int[numOrders];
        for (int o = 0; o < numOrders; o++) {
            totalUnitsPerOrder[o] =
                orders.get(o).values().stream().mapToInt(Integer::intValue).sum();
        }

        // 5. Restricción de tamaño de oleada: waveSizeLB ≤ sum(unidades·x) ≤ waveSizeUB
        {
            List<LinearExpr> terms = new ArrayList<>();
            for (int o = 0; o < numOrders; o++) {
                if (totalUnitsPerOrder[o] > 0) {
                    terms.add(LinearExpr.term(x[o], totalUnitsPerOrder[o]));
                }
            }
            LinearExpr waveExpr = LinearExpr.sum(terms.toArray(new LinearExpr[0]));
            model.addLinearConstraint(waveExpr, waveSizeLB, waveSizeUB);
        }

        // 6. Restricciones de disponibilidad por ítem: demand ≤ supply
        for (int i = 0; i < nItems; i++) {
            List<LinearExpr> demandTerms = new ArrayList<>();
            List<LinearExpr> supplyTerms = new ArrayList<>();
            for (int o = 0; o < numOrders; o++) {
                Integer q = orders.get(o).get(i);
                if (q != null && q > 0) {
                    demandTerms.add(LinearExpr.term(x[o], q));
                }
            }
            for (int a = 0; a < numAisles; a++) {
                Integer q = aisles.get(a).get(i);
                if (q != null && q > 0) {
                    supplyTerms.add(LinearExpr.term(y[a], q));
                }
            }
            LinearExpr demandExpr = LinearExpr.sum(demandTerms.toArray(new LinearExpr[0]));
            LinearExpr supplyExpr = LinearExpr.sum(supplyTerms.toArray(new LinearExpr[0]));
            model.addLessOrEqual(demandExpr, supplyExpr);
        }

        // 7. Función objetivo: maximizar bigM·sum(unidades·x) – sum(y)
        {
            List<LinearExpr> objTerms = new ArrayList<>();
            int bigM = 1_000;
            for (int o = 0; o < numOrders; o++) {
                if (totalUnitsPerOrder[o] > 0) {
                    objTerms.add(LinearExpr.term(x[o], totalUnitsPerOrder[o] * bigM));
                }
            }
            for (int a = 0; a < numAisles; a++) {
                objTerms.add(LinearExpr.term(y[a], -1));
            }
            LinearExpr objective = LinearExpr.sum(objTerms.toArray(new LinearExpr[0]));
            model.maximize(objective);
        }

        // 8. Resolución con límite de tiempo (en segundos)
        CpSolver solver = new CpSolver();
        solver.getParameters().setMaxTimeInSeconds(MAX_RUNTIME / 1000.0);
        CpSolverStatus status = solver.solve(model);

        // 9. Extracción de la solución
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
