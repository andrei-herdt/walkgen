#ifndef QP_WRAPPER_INL_H_
#define QP_WRAPPER_INL_H_

public:
//\name Accessors and mutators
//\{
inline QPMatrix &hessian_mat() {return hessian_mat_;}
inline QPMatrix &constr_mat() {return cstr_mat_;}

QPVector &vector(const QPVectorType type);

inline void num_var (int nbvar) {num_vars_ = nbvar;}
inline int num_var () const {return num_vars_;}
inline void num_var_max (int nbvar) {num_vars_max_ = nbvar;}
inline int num_var_max () const {return num_vars_max_;}
inline void num_constr(int num_constr) {num_constr_ = num_constr;}
inline int num_constr() const {return num_constr_;}
inline void num_constr_max(const int num_constr) {num_constr_max_ = num_constr;}
inline int num_constr_max() const {return num_constr_max_;}
//\}


#endif /* QP_WRAPPER_INL_H_ */
