import csv

# 設定：ここでxの補正値を決める（例: +1.0）
DELTA_X = 2.5

# 固定ファイル名
INPUT_CSV = 'raceline_awsim_15km_mod_org.csv'
OUTPUT_CSV = 'raceline_awsim_15km.csv'

def modify_x():
    with open(INPUT_CSV, 'r', newline='') as infile:
        reader = csv.reader(infile)
        header = next(reader)

        modified_rows = []
        for row in reader:
            if not row:
                continue
            row[0] = str(float(row[0]) + DELTA_X)  # x座標のみ修正（1列目）
            modified_rows.append(row)

    with open(OUTPUT_CSV, 'w', newline='') as outfile:
        writer = csv.writer(outfile)
        writer.writerow(header)
        writer.writerows(modified_rows)

    print(f"✅ xに{DELTA_X:+}を加えて保存しました → {OUTPUT_CSV}")

if __name__ == '__main__':
    modify_x()
