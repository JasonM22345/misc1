-- Start the transaction
START TRANSACTION;

-- Step 1: Delete everything from the `inventoryquantity` table
DELETE FROM tableslocalshelves.inventoryquantity;

-- Step 2: Recalculate quantities and insert them into the `inventoryquantity` table
INSERT INTO tableslocalshelves.inventoryquantity (isbn, isbncheck, TotalQuantity, NewQuantity, UsedQuantity, OtherQuantity, MissingQuantity)
SELECT 
    isbn,
    MAX(isbncheck) AS isbncheck, -- Select a representative isbncheck (e.g., the maximum value)
    COUNT(*) AS TotalQuantity,
    SUM(CASE WHEN `Condition` = 'New' THEN 1 ELSE 0 END) AS NewQuantity,
    SUM(CASE WHEN `Condition` = 'Used' THEN 1 ELSE 0 END) AS UsedQuantity,
    SUM(CASE WHEN `Condition` = 'Other' THEN 1 ELSE 0 END) AS OtherQuantity,
    SUM(CASE WHEN `Condition` = 'Missing' THEN 1 ELSE 0 END) AS MissingQuantity
FROM 
    tableslocalshelves.localbooksinventory
GROUP BY 
    isbn;

-- Commit the transaction
COMMIT;
