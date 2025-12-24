#include "mpc_ros2_pkg/MpcCal.h"

MpcCal::MpcCal() {
	status_node.clear();
	obs_vec.clear();
	MPCInit();
}

void MpcCal::MPCInit() {
	MatrixXd tmp = MatrixXd::Identity(N,N);

	A << 1, 0, DT, 0, 0.5 * DTT, 0, 
		 0, 1, 0, DT, 0, 0.5 * DTT, 
		 0, 0, 1, 0, DT, 0, 
		 0, 0, 0, 1, 0, DT, 
		 0, 0, 0, 0, 1, 0, 
		 0, 0, 0, 0, 0, 1;
	B << 0.5 * DTT, 0,
		0, 0.5 * DTT,
		DT, 0,
		0, DT,
		1, 0,
		0, 1;

	Q(0, 0) = 10;
	Q(1, 1) = 10;

	R *= 0.1;

	Q_bar = kroneckerProduct(tmp, Q);
	R_bar = kroneckerProduct(tmp, R);

	for (int i = 0; i < N; i++) {
		M.block(i*STANUM, 0, STANUM, M.cols()) = A.pow(i+1);

		for (int j = 0; j <= i; j++) {
			T.block(i*STANUM, j*CONNUM, STANUM, CONNUM) = A.pow(i-j)*B;
		}

	}

	H = T.transpose()*Q_bar*T+R_bar;
	H_sparse = H.sparseView();

	obs_vec.push_back(obs1);

	cout << "MPC init successful" << endl;

}

bool MpcCal::MPCSolver(VectorXd ref, VectorXd& sta_local) {
	RowVectorXd f = RowVectorXd::Zero(N*CONNUM);
	SparseMatrix<double> Acon_sparse;
	VectorXd ub = VectorXd::Zero(CONSROWNUM);
	VectorXd sta_ref = VectorXd::Zero(STANUM);
	VectorXd sta_ref_bar = VectorXd::Zero(N*STANUM);
	VectorXd u_sol = VectorXd::Zero(CONNUM);
	
    ConsCal(Acon_sparse, ub, sta_local);
    
    sta_ref[0] = ref[0];
    sta_ref[1] = ref[1];
    sta_ref_bar = sta_ref.replicate(N,1);

    f = (M*sta_local-sta_ref_bar).transpose()*Q_bar*T;

    OsqpEigen::Solver solver;

    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    solver.data()->setNumberOfVariables(N * CONNUM);
    solver.data()->setNumberOfConstraints(CONSROWNUM);

    if(!solver.data()->setHessianMatrix(H_sparse)) return false;
    if(!solver.data()->setGradient(f.transpose())) return false;
    if(!solver.data()->setLinearConstraintsMatrix(Acon_sparse)) return false;
    if(!solver.data()->setLowerBound(lb)) return false;
    if(!solver.data()->setUpperBound(ub)) return false;

    if(!solver.initSolver()) return false;

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;

    if (solver.getStatus() == OsqpEigen::Status::Solved) {
        u_sol = solver.getSolution().segment(0,CONNUM);
        sta_local = A * sta_local + B * u_sol;
        return true;
    }
	return false;
}

void MpcCal::ConsCal(SparseMatrix<double>& Acon_sparse, VectorXd& ub, VectorXd sta_local) {
	MatrixXd Acon = MatrixXd::Zero(CONSROWNUM, N*CONNUM);
	MatrixXd dire_bar = MatrixXd::Zero(CONSROWNUM, N*STANUM);
	RowVectorXd dire_single = VectorXd::Zero(STANUM);
	
	for (int i = 0; i < CONSROWNUM/N; i++) {
		dire_single.head(2) = (sta_local.head(2)-obs_vec[i]).normalized();
		dire_bar.block(i*N, 0, N, dire_bar.cols()) = kroneckerProduct(MatrixXd::Identity(N, N), -dire_single);
		
		double tmp = dire_single.head(2)*obs_vec[i]+safe_r+obs_r;
		ub.segment(i*N, N) = VectorXd::Constant(N,-tmp)-dire_bar*M*sta_local;
	}

	Acon = dire_bar*T;
	Acon_sparse = Acon.sparseView();
}