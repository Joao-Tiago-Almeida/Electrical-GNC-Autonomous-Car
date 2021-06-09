function pred_t = It_Prediction(size)
    global fixed_sample_rate
    d = size*fixed_sample_rate;
    pred_t = d/(1*0.1);
end