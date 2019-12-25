# extended_kalman_filter
Udacity Self Driving Nanodegree Kalman Filter project

- Check "RMSE values log.docx" file how the changes to the EKF affected the RMSE values.

### Few notes -
- tools.cpp needed std namespace without which cout and endl were throwing undefined errors
- If `Eigen::MatrixXd` operations had any mismatch in sizes, compilation would be fine but would run into 'core dump' and will be aborted. The error thrown when core dumped isnt intuitive. Used `gdb` and `backtrace` command to check which matrix throws the error
- Workspace constantly throws dependency issues. Run `./install-ubuntu.sh` to resolve them
- Normalization of the phi value between -pi and pi needed for the final RMSE value. 
- Improving precision by using doubles instead of floats did not improve the results. 
