import numpy as np
from sklearn.tree import DecisionTreeClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
import pickle
from pathlib import Path

# 人工的な学習データを作成
rng = np.random.default_rng(42)
X = rng.uniform(0.2, 5.0, (3000, 3))

# 入力は [左、前、右] の距離
y = np.where(
    X[:, 1] >= 0.8,
    0,
    np.where(X[:, 0] >= X[:, 2], 1, 2)
)

# 学習データと評価データに分割
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42,
    stratify=y
)

# 決定木モデルを学習
model = DecisionTreeClassifier(max_depth=8, random_state=42)
model.fit(X_train, y_train)

predictions = model.predict(X_test)
accuracy = accuracy_score(y_test, predictions)
print(f"AI accuracy: {accuracy * 100:.1f}%")

# 学習済みモデルを保存
output = Path.home() / "physical_ai_final/scripts/ai_model.pkl"
output.parent.mkdir(parents=True, exist_ok=True)

with output.open("wb") as f:
    pickle.dump(model, f)

print(f"AI model saved: {output}")
