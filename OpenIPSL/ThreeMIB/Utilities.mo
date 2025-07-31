within OpenIPSL.ThreeMIB;
package Utilities
  function saveTotalSMIBModel "Save the SMIB package as a total model"
  output Boolean ok "True if succesful";
  algorithm
  ok := saveTotalModel("SMIBTotal.mo", "SMIB", true);
  end saveTotalSMIBModel;
end Utilities;
