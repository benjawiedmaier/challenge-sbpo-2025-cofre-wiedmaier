package org.sbpo2025.challenge;

import org.apache.commons.lang3.time.StopWatch;
import java.util.*;
import java.util.concurrent.Future;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import ilog.concert.*;
import ilog.cplex.IloCplex;

public class ChallengeSolver {
    /* ─────────────────────────── Parámetros globales ─────────────────────────── */
    private final long MAX_RUNTIME = 585_000;            // 585 s  (~9:45 min)

    protected List<Map<Integer, Integer>> orders;
    protected List<Map<Integer, Integer>> aisles;
    protected int  nItems;
    protected int  waveSizeLB;
    protected int  waveSizeUB;

    public ChallengeSolver(List<Map<Integer, Integer>> orders,
                           List<Map<Integer, Integer>> aisles,
                           int nItems,
                           int waveSizeLB,
                           int waveSizeUB) {
        this.orders     = orders;
        this.aisles     = aisles;
        this.nItems     = nItems;
        this.waveSizeLB = waveSizeLB;
        this.waveSizeUB = waveSizeUB;
    }

    /* ────────────────────────────── MÉTODO PRINCIPAL ─────────────────────────── */
    public ChallengeSolution solve(StopWatch sw) {
        try {
            /* ---------- Pre-cálculos ---------- */
            int numOrders = orders.size();
            int[] unitsPerOrder = new int[numOrders];
            int   totalUnitsRequested = 0;
            for (int o = 0; o < numOrders; o++) {
                int sum = orders.get(o).values().stream().mapToInt(Integer::intValue).sum();
                unitsPerOrder[o] = sum;
                totalUnitsRequested += sum;
            }

            /* ---------- Gestión de tiempos ---------- */
            long startTime     = System.currentTimeMillis();
            final long GRASP_BUDGET  = (long)(0.15 * MAX_RUNTIME);   // 15 %
            final long RATIO_BUDGET  = (long)(0.60 * MAX_RUNTIME);   // 60 %
            final long RANDOM_BUDGET = MAX_RUNTIME - GRASP_BUDGET - RATIO_BUDGET;

            Random rand = new Random();

            /* ==================== 1)  GRASP + VND ==================== */
            long graspEnd = startTime + GRASP_BUDGET;
            ChallengeSolution bestSol  = null;
            double            bestRatio = 0.0;

            while (System.currentTimeMillis() < graspEnd) {
                ChallengeSolution g = constructiveGreedy(unitsPerOrder);
                ChallengeSolution v = vndLocalSearch(g, unitsPerOrder);
                if (!isSolutionFeasible(v)) continue;          // defensa extra
                double r = computeObjectiveFunction(v);
                if (r > bestRatio) {
                    bestRatio = r;
                    bestSol   = v;
                }
            }
            /* backup extremo */
            if (bestSol == null) {
                Set<Integer> fo = new HashSet<>(List.of(0));
                Set<Integer> fa = greedyAddAislesForOrder(0, new HashSet<>());
                bestSol   = new ChallengeSolution(fo, fa);
                bestRatio = computeObjectiveFunction(bestSol);
            }

            /* =================== 2)  Binary Search =================== */
            long ratioStart     = System.currentTimeMillis();
            long ratioDeadline  = ratioStart + RATIO_BUDGET;

            double left         = bestRatio;           // factible
            double right        = totalUnitsRequested; // infactible trivial cota sup

            while (System.currentTimeMillis() < ratioDeadline) {
                double mid = (left + right) / 2.0;
                double timeRemSec = (ratioDeadline - System.currentTimeMillis()) / 1000.0;
                if (timeRemSec <= 1.0) break;          // 1 s de colchón

                ChallengeSolution cand = solveForRatio(mid, unitsPerOrder, timeRemSec);
                if (cand != null) {                    // factible ⇒ subir la exigencia
                    double r = computeObjectiveFunction(cand);
                    if (r > bestRatio) {
                        bestRatio = r;
                        bestSol   = cand;
                    }
                    left = mid;
                } else {                               // infactible ⇒ bajar la exigencia
                    right = mid;
                }
            }

            /* =============== 3)  Búsqueda aleatoria final =============== */
            long randomDeadline = startTime + MAX_RUNTIME;
            while (System.currentTimeMillis() < randomDeadline) {
                ChallengeSolution cand = randomJump(bestSol, unitsPerOrder, rand);
                if (cand == null) continue;
                double r = computeObjectiveFunction(cand);
                if (r > bestRatio) {
                    bestRatio = r;
                    bestSol   = cand;
                }
            }

            return bestSol;   // siempre factible

        } catch (IloException ex) {
            throw new RuntimeException("Error en CPLEX: " + ex.getMessage(), ex);
        }
    }

    /* ────────────────────────  BÚSQUEDA ALEATORIA  ─────────────────────────── */
    /** Salto aleatorio sobre la solución actual.
     *  - 50 %: intenta añadir una orden aleatoria
     *  - 50 %: intenta eliminar una orden al azar (si se conserva factibilidad)
     */
    private ChallengeSolution randomJump(ChallengeSolution base,
                                         int[] unitsPerOrder,
                                         Random rand) {
        Set<Integer> candO = new HashSet<>(base.orders());
        if (rand.nextBoolean() && candO.size() < orders.size()) {
            /* añadir una orden no usada */
            List<Integer> free = new ArrayList<>();
            for (int o = 0; o < orders.size(); o++)
                if (!candO.contains(o)) free.add(o);
            Collections.shuffle(free, rand);
            for (int o : free) {
                candO.add(o);
                break;
            }
        } else if (candO.size() > 1) {
            /* quitar una orden al azar */
            List<Integer> list = new ArrayList<>(candO);
            int idx = rand.nextInt(list.size());
            candO.remove(list.get(idx));
        }

        /* reconstruye pasillos y comprueba factibilidad */
        Set<Integer> candA = new HashSet<>();
        int candUnits = 0;
        for (int o : candO) {
            candUnits += unitsPerOrder[o];
            candA.addAll(greedyAddAislesForOrder(o, candA));
        }
        if (candUnits < waveSizeLB || candUnits > waveSizeUB || candA.isEmpty() || candA.size() > 20)
            return null;   // descarte rápido

        ChallengeSolution cs = new ChallengeSolution(candO, candA);
        return isSolutionFeasible(cs) ? cs : null;
    }


    /* ──────────────────────  SUBMODELO: ratio mínimo  ─────────────────────── */
    private ChallengeSolution solveForRatio(double ratioConstraint,
                                            int[] unitsPerOrder,
                                            double timeLimitSec) throws IloException {

        int numOrders = orders.size();
        int numAisles = aisles.size();

        IloCplex cplex = new IloCplex();
        cplex.setParam(IloCplex.Param.Threads,
                       Runtime.getRuntime().availableProcessors());
        cplex.setParam(IloCplex.Param.TimeLimit, timeLimitSec);
        cplex.setOut(null);

        IloNumVar[] x = cplex.boolVarArray(numOrders);   // órdenes
        IloNumVar[] y = cplex.boolVarArray(numAisles);   // pasillos

        /* 1) waveSizeLB ≤ Σ u_o x_o ≤ waveSizeUB */
        IloLinearNumExpr totU = cplex.linearNumExpr();
        for (int o = 0; o < numOrders; o++) totU.addTerm(unitsPerOrder[o], x[o]);
        if (numOrders > 500) {
            IloLinearNumExpr numSelectedOrders = cplex.linearNumExpr();
            for (int o = 0; o < numOrders; o++) numSelectedOrders.addTerm(1.0, x[o]);
            double upperBound = Math.min(numOrders - 1, ((waveSizeUB + waveSizeLB) / 2.0) + (numAisles / 4.0));
            cplex.addLe(numSelectedOrders, upperBound);
        }
        cplex.addGe(totU, waveSizeLB);
        cplex.addLe(totU, waveSizeUB);

        /* 2) Σ y_a ≤ 20 */
        IloLinearNumExpr totA = cplex.linearNumExpr();
        for (int a = 0; a < numAisles; a++) totA.addTerm(1.0, y[a]);
        cplex.addLe(totA, 20);

        /* 3) Cobertura por ítem */
        for (int i = 0; i < nItems; i++) {
            IloLinearNumExpr demand = cplex.linearNumExpr();
            IloLinearNumExpr supply = cplex.linearNumExpr();
            for (int o = 0; o < numOrders; o++) {
                Integer q = orders.get(o).get(i);
                if (q != null && q > 0) demand.addTerm(q, x[o]);
            }
            for (int a = 0; a < numAisles; a++) {
                Integer q = aisles.get(a).get(i);
                if (q != null && q > 0) supply.addTerm(q, y[a]);
            }
            cplex.addLe(demand, supply);
        }

        /* 4) Productividad: Σ u_o x_o − ratio*Σ y_a ≥ 0 */
        IloLinearNumExpr prod = cplex.linearNumExpr();
        for (int o = 0; o < numOrders; o++) prod.addTerm(unitsPerOrder[o], x[o]);
        for (int a = 0; a < numAisles; a++) prod.addTerm(-ratioConstraint, y[a]);
        cplex.addGe(prod, 0.0);

        /* 5) Objetivo dummy */
        cplex.addMaximize(cplex.linearNumExpr());

        ChallengeSolution res = null;
        if (cplex.solve()) {
            Set<Integer> so = new HashSet<>();
            Set<Integer> sa = new HashSet<>();
            for (int o = 0; o < numOrders; o++)
                if (cplex.getValue(x[o]) > 0.5) so.add(o);
            for (int a = 0; a < numAisles; a++)
                if (cplex.getValue(y[a]) > 0.5) sa.add(a);
            res = new ChallengeSolution(so, sa);
        }
        cplex.end();
        return res;
    }

    /* ──────────────────────────  GRASP constructivo  ───────────────────────── */
    private ChallengeSolution constructiveGreedy(int[] unitsPerOrder) {
        Random rand = new Random();
        int numOrders = orders.size();

        Set<Integer> curO = new HashSet<>();
        Set<Integer> curA = new HashSet<>();

        /* 1) arranque aleatorio */
        int first = rand.nextInt(numOrders);
        curO.add(first);
        curA.addAll(greedyAddAislesForOrder(first, curA));
        int curUnits = unitsPerOrder[first];
        double curRatio = curUnits / (double) curA.size();

        /* 2) añadir mientras cumpla ratio */
        boolean improved = true;
        while (improved && curUnits < waveSizeLB) {
            improved = false;
            List<OrderDelta> deltas = new ArrayList<>();
            for (int o = 0; o < numOrders; o++) {
                if (curO.contains(o)) continue;
                int nu = curUnits + unitsPerOrder[o];
                if (nu > waveSizeUB) continue;
                Set<Integer> addA = greedyAddAislesForOrder(o, curA);
                double r = nu / (double)(curA.size() + addA.size());
                deltas.add(new OrderDelta(o, r, addA));
            }
            if (deltas.isEmpty()) break;
            deltas.sort((d1,d2) -> Double.compare(d2.ratio, d1.ratio));
            int k = Math.max(1, (int)(deltas.size()*0.3));    // RCL 30 %
            OrderDelta pick = deltas.get(rand.nextInt(k));
            if (pick.ratio > curRatio) {
                curO.add(pick.orderIdx);
                curA.addAll(pick.addedAisles);
                curUnits += unitsPerOrder[pick.orderIdx];
                curRatio  = pick.ratio;
                improved  = true;
            }
        }

        /* 3) reparación para asegurar LB */
        if (curUnits < waveSizeLB) {
            List<Integer> pend = new ArrayList<>();
            for (int o = 0; o < numOrders; o++)
                if (!curO.contains(o)) pend.add(o);
            pend.sort((o1,o2) -> unitsPerOrder[o2] - unitsPerOrder[o1]);
            for (int o : pend) {
                if (curUnits >= waveSizeLB) break;
                if (curUnits + unitsPerOrder[o] > waveSizeUB) continue;
                curO.add(o);
                curA.addAll(greedyAddAislesForOrder(o, curA));
                curUnits += unitsPerOrder[o];
            }
        }

        ChallengeSolution sol = new ChallengeSolution(curO, curA);
        if (!isSolutionFeasible(sol)) {       // fallback trivial
            Set<Integer> fo = new HashSet<>(List.of(0));
            Set<Integer> fa = greedyAddAislesForOrder(0, new HashSet<>());
            sol = new ChallengeSolution(fo, fa);
        }
        return sol;
    }

    /* ───────────────────────────  VND    ─────────────────────────── */
    private ChallengeSolution vndLocalSearch(ChallengeSolution init,
                                             int[] unitsPerOrder) {
        Set<Integer> bestO = new HashSet<>(init.orders());
        Set<Integer> bestA = new HashSet<>(init.aisles());
        double bestR = computeObjectiveFunction(init);
        boolean improv = true;

        while (improv) {
            improv = false;

            /* 1) intentar eliminar una orden */
            for (int o : new HashSet<>(bestO)) {
                Set<Integer> candO = new HashSet<>(bestO);
                candO.remove(o);
                Set<Integer> candA = new HashSet<>();
                int candU = 0;
                for (int oo : candO) {
                    candU += unitsPerOrder[oo];
                    candA.addAll(greedyAddAislesForOrder(oo, candA));
                }
                double r = candA.isEmpty() ? 0.0 : candU / (double)candA.size();
                if (r > bestR && candU >= waveSizeLB && candU <= waveSizeUB) {
                    bestO = candO; bestA = candA; bestR = r; improv = true; break;
                }
            }
            if (improv) continue;

            /* 2) intentar añadir una orden */
            for (int o = 0; o < orders.size(); o++) {
                if (bestO.contains(o)) continue;
                Set<Integer> candO = new HashSet<>(bestO); candO.add(o);
                int candU = 0;
                Set<Integer> candA = new HashSet<>(bestA);
                for (int oo : candO) {
                    candU += unitsPerOrder[oo];
                    candA.addAll(greedyAddAislesForOrder(oo, candA));
                }
                if (candU > waveSizeUB) continue;
                double r = candU / (double)candA.size();
                if (r > bestR && candU >= waveSizeLB) {
                    bestO = candO; bestA = candA; bestR = r; improv = true; break;
                }
            }
        }
        return new ChallengeSolution(bestO, bestA);
    }

    /* ─────────────────── Greedy para cubrir una orden ─────────────────── */
    private Set<Integer> greedyAddAislesForOrder(int orderIdx,
                                                 Set<Integer> currentAisles) {
        Set<Integer> added = new HashSet<>();
        Map<Integer,Integer> demand = orders.get(orderIdx);

        for (Map.Entry<Integer,Integer> e : demand.entrySet()) {
            int item = e.getKey(), qty = e.getValue();
            if (qty <= 0) continue;

            /* verificar cobertura actual */
            int covered = 0;
            for (int a : currentAisles) {
                covered += aisles.get(a).getOrDefault(item, 0);
            }
            if (covered >= qty) continue;

            /* elegir pasillo con mejor suministro */
            int bestA = -1, bestSup = 0;
            for (int a = 0; a < aisles.size(); a++) {
                if (currentAisles.contains(a) || added.contains(a)) continue;
                int sup = aisles.get(a).getOrDefault(item, 0);
                if (sup > bestSup) { bestSup = sup; bestA = a; }
            }
            if (bestA >= 0) added.add(bestA);
        }
        return added;
    }

    /* ───────────────────────  Utilidades varias ─────────────────────── */
    private static class OrderDelta {
        final int orderIdx;
        final double ratio;
        final Set<Integer> addedAisles;
        OrderDelta(int orderIdx, double ratio, Set<Integer> addedAisles) {
            this.orderIdx = orderIdx; this.ratio = ratio; this.addedAisles = addedAisles;
        }
    }

    protected boolean isSolutionFeasible(ChallengeSolution sol) {
        Set<Integer> selO = sol.orders();
        Set<Integer> selA = sol.aisles();
        if (selO == null || selA == null || selO.isEmpty() || selA.isEmpty()) return false;

        int[] picked   = new int[nItems];
        int[] supplied = new int[nItems];

        for (int o : selO)
            for (var e : orders.get(o).entrySet()) picked[e.getKey()] += e.getValue();

        for (int a : selA)
            for (var e : aisles.get(a).entrySet()) supplied[e.getKey()] += e.getValue();

        int totUnits = Arrays.stream(picked).sum();
        if (totUnits < waveSizeLB || totUnits > waveSizeUB || selA.size() > 20) return false;
        for (int i = 0; i < nItems; i++)
            if (picked[i] > supplied[i]) return false;
        return true;
    }

    protected double computeObjectiveFunction(ChallengeSolution sol) {
        Set<Integer> selO = sol.orders();
        Set<Integer> selA = sol.aisles();
        if (selO == null || selA == null || selO.isEmpty() || selA.isEmpty()) return 0.0;
        int units = selO.stream().mapToInt(o -> orders.get(o).values().stream()
                                     .mapToInt(Integer::intValue).sum()).sum();
        return units / (double) selA.size();
    }
}