% TODO: validate function

function testResult = compareFunctionOutputs(func1, func2, varargin)
% compareFunctionOutputs Compares the outputs of two functions.
%   This function takes two function handles and a set of arguments,
%   then compares the outputs of these functions using the same arguments.
%
% Inputs:
%   func1 - handle to the first function
%   func2 - handle to the second function
%   varargin - variable input arguments passed to the functions
%
% Output:
%   testResult - boolean result of the test (true if outputs are the same)

    try
        % Get outputs from the first function
        [outputs1{1:nargout(func1)}] = func1(varargin{:});

        % Get outputs from the second function
        [outputs2{1:nargout(func2)}] = func2(varargin{:});

        % Compare the outputs
        testResult = isequal(outputs1, outputs2);

        if testResult
            fprintf('Test Passed: The outputs are the same.\n');
        else
            fprintf('Test Failed: The outputs are different.\n');
        end
    catch err
        % Handle any error that occurred during the test
        disp('An error occurred during testing:');
        disp(err.message);
        testResult = false;
    end
end
