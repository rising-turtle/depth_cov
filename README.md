# depth_cov
characterize depth cov for structure core sensor 

## shell 

run shell/run_solve_lambda_folder.sh to obtain the results given the lambda for depth defined in line 20  

run shell/run_solve_lambda_folder_inverse.sh to obtain the results given the lambda for inverse depth defined in line 19

Notice to change the parameters (exe_dir and folder_name) in the *.sh* files to fit your environment 

## results 

under the **train_test_results**: 

**train_depth**: contains the depth images [x.png] and the Monte Carlo estimated depth std of each pixel [x.exr]  
**train_inv_depth**: contains the depth images [x.png] and the Monte Carlo estimated inverse depth std of each pixel [x.exr]   
**test_d**: estimated depth std using prediction (central point fitting), GMM, and GMM-extend   
**test_inv_d**: estimated inverse depth std using prediction (const value), GMM, and GMM-extend  
