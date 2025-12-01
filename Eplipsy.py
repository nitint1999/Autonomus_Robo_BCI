# Test Code
import RPi.GPIO as GPIO
import time

import pandas as pd
import joblib
GPIO.setmode(GPIO.BOARD)

led = 3

GPIO.setup(led,GPIO.OUT)

data = pd.read_excel('/home/naveen/Desktop/LED/SPANDANA.xlsx')
df = pd.DataFrame(data)
df = df.drop(['Time'], axis=1)
df = df.transpose()
x = df.iloc[:, :-1]
y = df.iloc[:, -1]

print("\n--- Test Case: Prediction with the Saved Model ---")

# Specify the row number for the sample query (e.g., n=12 for the 12th row, as Python is 0-indexed)
n = 12

# Create a test case by selecting the specified row from X
sample_query_column = x.iloc[n:n+1] # Selects row 'n' as a DataFrame slice
print(f"Sample query column (row {n} of x):\n{sample_query_column}")

# Load the saved model
loaded_model = joblib.load('/home/naveen/Desktop/LED/decision_tree_model.joblib')
print(f"Model loaded successfully")

# Make a prediction using the loaded model
prediction = loaded_model.predict(sample_query_column)

print(f"Predicted 'Y' label for the sample query: {prediction[0]}")

while (prediction[0]==1):

    GPIO.output(led,True)
    time.sleep(0.1)
    GPIO.output(led,False)
    time.sleep(0.3)