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
        // 1. Crear el solver MIP (CBC)
        MPSolver solver = MPSolver.createSolver("CBC_MIXED_INTEGER_PROGRAMMING"); // SCIP_MIXED_INTEGER_PROGRAMMING 
        if (solver == null) {
            throw new IllegalStateException("No se pudo crear el solver CBC");
        }

        int numOrders = orders.size();
        int numAisles = aisles.size();

        // 2. Variables binarias
        MPVariable[] x = new MPVariable[numOrders];      // x[o] = 1 si tomo la orden o
        MPVariable[] y = new MPVariable[numAisles];      // y[a] = 1 si visito el pasillo a
        for (int o = 0; o < numOrders; o++) {
            x[o] = solver.makeBoolVar("x_o_" + o);
        }
        for (int a = 0; a < numAisles; a++) {
            y[a] = solver.makeBoolVar("y_a_" + a);
        }

        // 3. Precomputar unidades totales por orden
        int[] totalUnitsPerOrder = new int[numOrders];
        for (int o = 0; o < numOrders; o++) {
            totalUnitsPerOrder[o] =
                orders.get(o).values().stream().mapToInt(Integer::intValue).sum();
        }

        // 4. Restricciones de disponibilidad por ítem
        for (int i = 0; i < nItems; i++) {
            MPConstraint ct = solver.makeConstraint(-MPSolver.infinity(), 0.0, "item_" + i);
            // suma de órdenes
            for (int o = 0; o < numOrders; o++) {
                Integer q = orders.get(o).get(i);
                if (q != null && q > 0) {
                    ct.setCoefficient(x[o], q);
                }
            }
            // menos suma de pasillos disponibles
            for (int a = 0; a < numAisles; a++) {
                Integer q = aisles.get(a).get(i);
                if (q != null && q > 0) {
                    ct.setCoefficient(y[a], -q);
                }
            }
        }

        // 5. Restricción de tamaño de oleada
        MPConstraint waveCt =
            solver.makeConstraint(waveSizeLB, waveSizeUB, "wave_size");
        for (int o = 0; o < numOrders; o++) {
            waveCt.setCoefficient(x[o], totalUnitsPerOrder[o]);
        }

        // 6. Función objetivo aproximada: max totalUnits*bigM - numPasillos
        //    (buscamos un buen equilibrio unidades/pasillo)
        MPObjective obj = solver.objective();
        int bigM = 1_000; // peso para priorizar unidades
        for (int o = 0; o < numOrders; o++) {
            obj.setCoefficient(x[o], totalUnitsPerOrder[o] * bigM);
        }
        for (int a = 0; a < numAisles; a++) {
            obj.setCoefficient(y[a], -1.0);
        }
        obj.setMaximization();

        // 7. Fijar límite de tiempo
        solver.setTimeLimit(599000); 
        solver.setSolverSpecificParametersAsString("threads=8\n");

        // 8. Ejecutar
        ResultStatus status = solver.solve();

        // 9. Construir la solución
        Set<Integer> selectedOrders = new HashSet<>();
        Set<Integer> visitedAisles = new HashSet<>();
        if (status == ResultStatus.OPTIMAL || status == ResultStatus.FEASIBLE) {
            for (int o = 0; o < numOrders; o++) {
                if (x[o].solutionValue() > 0.5) {
                    selectedOrders.add(o);
                }
            }
            for (int a = 0; a < numAisles; a++) {
                if (y[a].solutionValue() > 0.5) {
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
