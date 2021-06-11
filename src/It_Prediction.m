function pred_t = It_Prediction(size)
    global fixed_sample_rate
    %a good prediction for the number of iterations is how many iterations
    %would it take for the car to go through the whole path in the minimum
    %velocity
    d = size*fixed_sample_rate;
    pred_t = d/(1*0.1);
end