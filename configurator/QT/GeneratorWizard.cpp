#include "GeneratorWizard.h"

GeneratorWizard::GeneratorWizard(QMainWindow* UNUSED(parent)) {
	worldModel = new WorldTableModel();
	setupUi(this);

	objectTable->setModel(worldModel);
}

void GeneratorWizard::evaluateButtonsState() {
	if (worldModel->rowCount() > 0) {
		okButton->setEnabled(true);
		editItemButton->setEnabled(true);
		deleteItemButton->setEnabled(true);
	} else {
		okButton->setEnabled(false);
		editItemButton->setEnabled(false);
		deleteItemButton->setEnabled(false);
	}
}

void GeneratorWizard::on_addNodesButton_clicked() {
	printf("add button clicked\n");
	Node nodeDialog;
	QStandardItemModel *model = new QStandardItemModel(1, 2);

	QModelIndex index = model->index(0, 0, QModelIndex());
	model->setData(index, "BaseMac");
	index = model->index(0, 1, QModelIndex());
	model->setData(index, "BasePhy");

	nodeDialog.setModel(model);

	if (nodeDialog.exec() == QDialog::Accepted) {
		worldModel->insertRow(1);
		worldModel->insertData(nodeDialog.getModel());
		//objectTable->setItem(0, 0, new QTableWidgetItem(QString::number(nodeDialog.getCount())));
		//objectTable->setItem(0, 1, new QTableWidgetItem(nodeDialog.getType()));
		//objectTable->setItem(0, 2, new QTableWidgetItem(nodeDialog.getApplicationName()));
		//printf("User selected %d nodes type %s with %s running %s\n", nodeDialog.getCount(), nodeDialog.getType().toLatin1(), nodeDialog.getNetworkLayer().toLatin1(), nodeDialog.getApplicationName().toLatin1());
        }
	evaluateButtonsState();
}

void GeneratorWizard::on_deleteItemButton_clicked() {
	//int curRow = objectTable->currentRow();
	//printf("Row about to be deleted: %d\n", curRow);
	//objectTable->removeRow(curRow);
	evaluateButtonsState();
}

void GeneratorWizard::on_cancelButton_clicked() {
	exit(0);
}

void GeneratorWizard::on_okButton_clicked() {
	close();
}

